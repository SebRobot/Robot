/*
 * PositionManager.cpp
 *
 *  Created on: 13 janv. 2016
 *      Author: seb
 */

#include <PositionManager.h>

#include "PropNetworkManager.h"

#include <iostream>

PositionManager::PositionManager(): simu(true) {
}

PositionManager::PositionManager(bool simu_init): simu(simu_init) {
}

PositionManager::~PositionManager() {
    // TODO Auto-generated destructor stub
}

void PositionManager::setPoint(PointOrient2D<float> &po, Vector2D<float> &v, float omega) {
    if (simu) {
        pos_cur = po;
        return;
    }

    std::cout << "Pas simu" << std::endl;

    // TODO add pid

    // convert in robot frame
    v.rotate(-offset_angle);

    pnm.sendSetPointSpeed(v, omega);
}

void PositionManager::setPosition(PointOrient2D<float> &pos_pg) {

    //FIXME si on a pas de pos_last of the robot (assert : le lien avec la prop doit etre établi et opérationnel)
/* FIXME
    Vector2D<float> op(pos_robot.p, pos_pg);
    offset_pos =  op;
    offset_angle = pos_pg.o - pos_robot.o;

    pos_cur = pos_pg;
    */
}

void PositionManager::updatePosition(PointOrient2D<float> &pos, Frame frame) {
    PointOrient2D<float> pos_pg; // playground position

    switch (frame) {
        case ROBOT:
            pos_pg.p = pos.p + offset_pos;
            pos_pg.o = pos.o + offset_angle;
            break;

        case PLAYGROUND:
            pos_pg = pos;
            break;
    }

    // TODO Check consistency of the new position

    pos_cur = pos_pg;
}


PointOrient2D<float> PositionManager::getPosition() {
    return pos_cur;
}
