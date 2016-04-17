/*
 * TrajectoryManager.cpp
 *
 *  Created on: 11 janv. 2016
 *      Author: Sebastien Malissard
 */

#include <TrajectoryManager.h>

#include <iostream>

#include "Tools.h"

extern "C" {
#include "millis.h"
}


using namespace std;

TrajectoryManager::TrajectoryManager(): cur_tid(0), cur_sid(0), cur_el(SEGMENT), cur_state(TM_STATE_WAIT_TRAJ) {
}

TrajectoryManager::~TrajectoryManager() {
}

void TrajectoryManager::newTrajEl(const sTrajOrientElRaw_t &s) {
    sTrajSlot ts;

    convertTrajOrientElRaw2TrajSlot(s, ts, 0);
    pushNewTrajEl(ts);
    newTrajSlot(ts);


    if (!s.elts[0].is_last_element) {
        convertTrajOrientElRaw2TrajSlot(s, ts, 1);
        pushNewTrajEl(ts);
        newTrajSlot(ts);
    }


}

sTrajSlot& TrajectoryManager::getCurSlot() {
    updateSlots();
    if (slots.front().tid == cur_tid && slots.front().sid == cur_sid) {
        return slots.front();
    }
    return ts_null;
}
sTrajSlot& TrajectoryManager::getNextSlot() {
    updateSlots();

    auto next = ++(slots.begin());

    if (next->tid == cur_tid && next->sid == cur_sid + 1) {
        return *next;
    }
    return ts_null;
}

void TrajectoryManager::updateSlots() {
    while (slots.front().tid < cur_tid) {
        slots.pop_front();
    }
    while (slots.front().tid == cur_tid && slots.front().sid < cur_sid) {
        slots.pop_front();
    }
}

void TrajectoryManager::newTrajSlot(const sTrajSlot &ts) {
    if (ts.sid == 0) { // new trajectory
        cur_tid = ts.tid;
        cur_sid = 0;
        cur_el = SEGMENT;
        cur_state = TM_STATE_WAIT_START;
    }
}

void TrajectoryManager::update() {
    PointOrient2D<float> po_sp; // sp = set point
    Vector2D<float> v_sp;
    float omega_sp = 0;
    float t = micros(); // FIXME (must be synchronized) current time (in us)

    switch (cur_state) {
        case TM_STATE_IDLE:
            return;

        case TM_STATE_WAIT_TRAJ:
            pm.stop();
            return;

        case TM_STATE_WAIT_START: {
            sTrajSlot s = getCurSlot();

            if (t - s.t1 <= 0) { // wait at the starting position
                pm.positionKeeping({s.p1, s.theta1});
                return;
            }

            cur_state = TM_STATE_FOLLOWING; // time is ok, let's go
        }
        /* no break */

        case TM_STATE_FOLLOWING: {
            sTrajSlot s;
            sTrajSlot s_next;

            while (1) {
                s = getCurSlot();
                s_next = getNextSlot();

                switch (cur_el) {
                    case ARC:
                        if (s_next.isNull() || s.last_element) { // wait the next element
                            //cur_state = TM_STATE_WAIT_TRAJ;
                            pm.positionKeeping({s.p2, s.theta2});
                            cout << "Waiting t="<< t << endl;
                            return;
                        }
                        else if (t - s_next.t1 >= 0) { // switch to next slot's segment
                            cout << "Switch vers segment t="<< t << endl;
                            cur_el = SEGMENT;
                            cur_sid++;
                            continue;
                        }
                        break;

                    case SEGMENT:
                        if (t - s.t2 >= 0) { // switch to arc
                            cur_el = ARC;
                            cout << "Switch vers arc t=" << t << endl;
                            continue;
                        }
                        break;
                    }

                break;
            }

            // note: When we received a message, wee look if we can update the previous.

            float dur;     // duration since start of element (in s)
            float elt_dur; // total duration of element (in s)
            float dtheta;  // delta theta for the element (theta_end - theta_start) (in rad)
            bool dir;      // direction of rotation (false: negative angle / ClockWise, true: positive angle / CounterClockWise)
            float theta0;  // initial orientation at start of element (in rad)

            switch (cur_el) {
                case ARC: {
                    dur = t - s.t2;

                    float alpha = dur * s.omega; // angle already done
                    Vector2D<float> v(s.c, s.p2);

                    cout << "ALPHA=" << alpha * 180 / M_PI << endl;

                    v.rotate(alpha);

                    po_sp.p = s.c + v;
                    v_sp.x = s.omega * v.y;
                    v_sp.y = s.omega * v.x;

                    dir = s.rot2_dir;
                    dtheta = s_next.theta1 - s.theta2;
                    theta0 = s.theta2;

                    elt_dur = s.arc_dur;
                    cout << "arc_dur=" << s.arc_dur << endl;

                    break;
                }

                case SEGMENT: {
                    dur = t - s.t1;

                    po_sp.p = s.p1 + s.v * dur;
                    v_sp = s.v;

                    dir = s.rot1_dir;
                    dtheta = s.theta2 - s.theta1;
                    theta0 = s.theta1;

                    elt_dur = s.seg_dur;

                    break;
                }
            }

            // ensures dtheta is of the right sign
            if (dir) {
                while (dtheta < 0) {
                    dtheta += 2 * M_PI; // dtheta must be positive
                }
            }
            else {
                while (dtheta > 0) {
                    dtheta -= 2 * M_PI; // dtheta must be negative
                }
            }

            po_sp.o = elt_dur ? theta0 + dtheta * dur / elt_dur : theta0;
            omega_sp = elt_dur ? dtheta / elt_dur : 0;

            break;
        }
    }

    cout << "po_sp.x=" << po_sp.p.x << "po_sp.y=" << po_sp.p.y << endl;

    pm.setPoint(po_sp, v_sp, omega_sp);
}


void TrajectoryManager::updatePreviousSlot(sTrajSlot &prev, sTrajSlot &cur) {
    prev.arc_dur = cur.t1 - prev.t2;
    prev.arc_spd = prev.arc_len / prev.arc_dur;

    cerr << "arc_spd=" << prev.arc_spd << " prev.r=" << prev.r << endl;

    prev.omega = prev.arc_spd / prev.r;
}

void TrajectoryManager::pushNewTrajEl(sTrajSlot &el) {

    if (slots.empty()) { // First element
        slots.push_back(el);
        return;
    }

    if (slots.back().tid == el.tid) {
        if (slots.back().sid + 1 == el.sid) { // Next element of the trajectory
            updatePreviousSlot(slots.back(), el);
            slots.push_back(el);
            return;
        }
    }
    else if (slots.back().tid != el.tid) { // New trajectory FIXME modulo
        if (el.sid == 0) {
            slots.push_back(el);
            return;
        }
    }

    cerr << "Error: trajectory element not add" << endl;

    return;

    cerr << "Warning: overwrite trajectory element" << endl;
}

void TrajectoryManager::convertTrajOrientElRaw2TrajSlot(const sTrajOrientElRaw_t &s, sTrajSlot &d, unsigned int ssid) {
    d.tid = s.tid;
    d.sid = 2 * s.sid + ssid;

    d.last_element = s.elts[ssid].is_last_element;

    d.t1 = s.t + (ssid ? (s.dt1 + s.dt2) * 1000 : 0);
    d.t2 = s.t + (s.dt1 + (ssid ? (s.dt2 + s.dt3) : 0)) * 1000;

    d.p1.x = 10 * FixedPoint2Double(s.elts[ssid].p1_x, 6);
    d.p1.y = 10 * FixedPoint2Double(s.elts[ssid].p1_y, 6);
    d.p2.x = 10 * FixedPoint2Double(s.elts[ssid].p2_x, 6);
    d.p2.y = 10 * FixedPoint2Double(s.elts[ssid].p2_y, 6);

    d.c.x = 10 * FixedPoint2Double(s.elts[ssid].c_x, 6);
    d.c.y = 10 * FixedPoint2Double(s.elts[ssid].c_y, 6);
    d.r = -10 * FixedPoint2Double(s.elts[ssid].c_r, 5);

    d.theta1 = FixedPoint2Double(s.elts[ssid].theta1, 13);
    d.theta2 = FixedPoint2Double(s.elts[ssid].theta2, 13);

    d.rot1_dir = s.elts[ssid].rot1_dir;
    d.rot2_dir = s.elts[ssid].rot2_dir;

    d.arc_len = 10 * FixedPoint2Double(s.elts[ssid].arc_len, 5);
    d.seg_len = 10 * FixedPoint2Double(s.elts[ssid].seg_len, 5);

    // additional data
    d.seg_dur = d.t2 - d.t1;
    d.seg_spd =  d.seg_len / d.seg_dur;

    Vector2D<float> v(d.p1, d.p2), vx(1, 0);
    d.v.x = d.seg_spd * cos(vx.angle(v));
    d.v.y = d.seg_spd * sin(vx.angle(v));

    cerr << "dvx=" << d.v.x << "dvy=" << d.v.y << "angle=" << vx.angle(v)*180/M_PI << endl;
    cerr << "seg_spd=" << d.seg_spd << " d.r=" << d.r << endl;
    cerr << "d.arc_len=" << d.arc_len << "d.seg_len=" << d.seg_len << endl;
}
