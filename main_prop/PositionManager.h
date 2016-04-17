/*
 * PositionManager.h
 *
 *  Created on: 13 janv. 2016
 *      Author: seb
 */

#ifndef POSITIONMANAGER_H_
#define POSITIONMANAGER_H_

#include "GeometryTools.h"

class PositionManager {
    public:
        PositionManager();
        PositionManager(bool simu_init);
        ~PositionManager();

        enum Frame {
            ROBOT,
            PLAYGROUND
        };

        void stop(){}; // stop the robot at the current position TODO maintien la premiere position appellé
        void positionKeeping(const PointOrient2D<float> &p){}; // keeping this position ; the robot must already be present around this position

        void setPoint(PointOrient2D<float> &po, Vector2D<float> &v, float omega);


        void setPosition(PointOrient2D<float> &pos_pg);                 // set a new position in the playground frame
        void updatePosition(PointOrient2D<float> &pos, Frame frame);    // update the new position of the robot

        PointOrient2D<float> getPosition();


        bool simu;

    private:

        PointOrient2D<float> pos_last;  // derniere consigne envoyer
        PointOrient2D<float> pos_cur;   // current position in the playground frame
        PointOrient2D<float> pos_robot; // current position in the local frame robot

        Vector2D<float> offset_pos;     // position offset (robot to playground)
        float offset_angle;             // angle offset (robot to playground)

};

#endif /* POSITIONMANAGER_H_ */
