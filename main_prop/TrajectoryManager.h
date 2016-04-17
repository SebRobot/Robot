/*
 * TrajectoryManager.h
 *
 *  Created on: 11 janv. 2016
 *      Author: Sebastien Malissard
 */

#ifndef TRAJECTORYMANAGER_H_
#define TRAJECTORYMANAGER_H_

#include <deque>

#include "GeometryTools.h"
#include "messages-locomotion.h"
#include "PositionManager.h"


/*
 * 1 segment 2
 * X---------X-\
 *           arc \
 *          c X    X
 */

struct sTrajSlot{ //TODO Integrer dans la classe
    sTrajSlot(): tid(0), sid(0), last_element(false), t1(0), t2(0) {};

    unsigned int tid;
    unsigned int sid;

    bool last_element;

    float t1; // in s (synchronized time):
    float t2;

    Point2D<float> p1; // in mm
    Point2D<float> p2;

    Point2D<float> c;
    float r; // in mm

    float theta1;
    float theta2;

    bool rot1_dir; // (false: CW | true: CCW)
    bool rot2_dir;

    float arc_len;
    float seg_len;

    float arc_spd;
    float seg_spd;

    /** no compute FIXME **/
    Vector2D<float> v; // segment velocity
    float omega; // arc velocity (<0 ClockWise | >0 CounterClockWise) (in rad/s)

    float arc_dur; //duration of the segment (in s)
    float seg_dur; //duration of the arc (in s)



    bool isNull(){
        if (!tid && !sid && !last_element && !t1 && !t2) //TODO continue
            return true;
        return false;
    }
};

using namespace std;
class TrajectoryManager {
    public:
        TrajectoryManager();
        ~TrajectoryManager();

        void newTrajEl(const sTrajOrientElRaw_t &te);
        void newTrajSlot(const sTrajSlot &ts);
        void update();

        void convertTrajOrientElRaw2TrajSlot(const sTrajOrientElRaw_t &s, sTrajSlot &d, unsigned int ssid);
        void pushNewTrajEl(sTrajSlot &el);
        void updatePreviousSlot(sTrajSlot &prev, sTrajSlot &cur);

        sTrajSlot& getCurSlot();
        sTrajSlot& getNextSlot();

        void updateSlots();

    private:
        std::deque<sTrajSlot> slots;

        sTrajSlot ts_null;

        unsigned int cur_tid;
        unsigned int cur_sid;
        enum {SEGMENT, ARC} cur_el;
        enum {
            TM_STATE_IDLE, // no control loop on position
            TM_STATE_WAIT_TRAJ, // no action asked (we are stopped)
            TM_STATE_WAIT_START, // new trajectory received, waiting for the right time to start
            TM_STATE_FOLLOWING // we are following a trajectory
        } cur_state; // state of the trajectory follow
};

#endif /* TRAJECTORYMANAGER_H_ */
