/*
 * PropNetworkManager.h
 *
 *  Created on: 14 janv. 2016
 *      Author: Sebastien Malissard
 *
 *
 * Messages received (8* bytes):
 * --------------------------------------------------------------
 * | START |       x      |       y      |     theta    |  END  |
 * --------------------------------------------------------------
 * x : -32768 to 32767 (in mm)
 * y : -32768 to 32767 (in mm)
 * theta : 0 to 62831 (in rad/10000)
 *
 * Messages sent (8* bytes):
 * --------------------------------------------------------------
 * | START |      v_x     |      v_y     |     theta    |  END  |
 * --------------------------------------------------------------
 * v_x : -32768 to 32767 (in mm/s)
 * v_y : -32768 to 32767 (in mm/s)
 * theta : -32768 to 32767 (in rad/s/1000)
 *
 * * Can have more bytes if there are some specials bytes
 *
 *  //TODO do with dynamic size parameter
 */

#ifndef PROPNETWORKMANAGER_H_
#define PROPNETWORKMANAGER_H_

#include <queue>

#include "Serial.h"
#include "GeometryTools.h"


class PropNetworkManager : public Serial{
    public:
        PropNetworkManager();
        ~PropNetworkManager();

        void update();                                              // update if new message received
        void sendSetPointSpeed(Vector2D<float> v, float theta);     // send speed in the robot frame

        PointOrient2D<float> getPosition();                         // get the position robot frame

        bool isNewPositionAvailable();                              // true if new position available
        bool isPositionAvailable();                                 // true if at least one position was received from the robot

        class NoPositionAvailable : public std::logic_error {
            public:
                NoPositionAvailable(const std::string& whatArg) : logic_error("NoPositionAvailable : " + whatArg) {
                }
            };

    private:
        void cleanBeginnigFIFO();

        struct data {
            uint8_t byte;
            bool    special_byte;
        };

        std::queue<data> fifo;          // fifo queue of the byte received
        PointOrient2D<float> pos;       // last know position in the robot frame

        bool new_pos_available;         // indicate if a new position is available
        bool pos_available;             // indicate if a position was received
        bool backslash;                 // indicate that the next byte is not a special byte

};

#endif /* PROPNETWORKMANAGER_H_ */

extern PropNetworkManager pnm;
