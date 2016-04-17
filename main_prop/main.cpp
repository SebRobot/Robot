/*
 * main.cpp
 *
 *  Created on: 14 nov. 2015
 *      Author: seb
 */


#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "shared/botNet_core.h"
#include "messages.h"

extern "C" {
#include <stdlib.h>

#include "roles.h"
#include "millis.h"
#include "global_errors.h"
}

#include "TrajectoryManager.h"
#include "PositionManager.h"
#include "Tools.h"
#include "Serial.h"
#include "PropNetworkManager.h"

using namespace std;

int main(int argc, char **argv) {
    sMsg inMsg, outMsg;
    int ret, i = 0;
    unsigned int prevPos_ms = millis(), prev_ctrl = millis();
    bool firstPosReceived = false;

    unsigned char byte = 0;
    PointOrient2D<float> pos;

    Serial ser("/dev/ttyACM0", Serial::BAUD_9600, Serial::CONFIG_8N1);

    ser.Open();

    while (1) {
        try {
        ser.ReadByte(&byte);
        }
        catch (Serial::NotEnoughData&) {
            continue;
        }
        printf("%02x", byte);
        if (byte == 0xff) {
            printf("\n");
        }
    }



/*
    while (1) {
        pnm.update();

        try {
            pos = pnm.getPosition();
        }
        catch (PropNetworkManager::NoPositionAvailable&){
            continue;
        }
        printf("x=%f y=%f theta=%f/%f\n", pos.p.x, pos.p.y, pos.o * 180 / M_PI, pos.o);
    }

    if (*argv[1] == 's') {
        std::cout << "SIMU" << std::endl;
    }

*/

    if ((ret = bn_attach(E_ROLE_SETUP, role_setup)) < 0) {
        cerr << "ERROR : bn_attach() : " << getErrorStr(-ret) << " (" << -ret << ")" << endl;
        exit(EXIT_FAILURE);
    }

    /*** Warning: No guaranteed synchronized time (must be execute on the same pc)***/

    if ((ret = bn_init()) < 0) {
        cerr << "ERROR : bn_init() : " << getErrorStr(-ret) << " (" << -ret << ")" << endl;
        exit(EXIT_FAILURE);
    }

    while (1) {
        ret = bn_receive(&inMsg);

        if (ret < 0) {
            cerr << "ERROR : bn_receive() : " << getErrorStr(-ret) << " (" << -ret << ")" << endl;
        }
        else if (ret > 0) {
            /*** Warning: No relaying of messages received ***/

            switch (inMsg.header.type) {
            case E_TRAJ_ORIENT_EL: // Get the new step of a trajectory
                printf("got traj elt: tid%hu, sid%hhu\n", (uint16_t)inMsg.payload.trajOrientEl.tid, (uint8_t)inMsg.payload.trajOrientEl.sid);
                tm.newTrajEl(inMsg.payload.trajOrientEl);
                firstPosReceived = true;
                break;
            case E_GENERIC_POS_STATUS:
                printf("got position: %.2fcm %.2fcm %.2f°\n", inMsg.payload.genericPosStatus.pos.x, inMsg.payload.genericPosStatus.pos.y, inMsg.payload.genericPosStatus.pos.theta * 180. / M_PI);
                /*
                switch(inMsg.payload.genericPosStatus.prop_status.action){
                case PROP_SETPOS:
                    trajmngr_set_pos(&traj_mngr, &inMsg.payload.genericPosStatus);
                    firstPosReceived = 1;
                    break;
                case PROP_MIXPOS:
                    trajmngr_mix_pos(&traj_mngr, &inMsg.payload.genericPosStatus);
                    break;
                default:
                    break;
                }
                */
                break;
            case E_PROP_STOP:
                printf("got prop stop\n");
                pm.stop();
                break;
            default:
                printf("got unhandled message with type (%i)\n", inMsg.header.type);
                break;
            }
        }

        //Periodic control
        if (millis() - prev_ctrl >= 100) {
            tm.update();

            prev_ctrl = millis();
        }

        // Periodic position send
        if (millis() - prevPos_ms >= 100) {
            if (firstPosReceived) {
                prevPos_ms = millis();

                memset(&outMsg, 0, sizeof(outMsg));

                outMsg.header.type = E_GENERIC_POS_STATUS;
                outMsg.header.size = sizeof(outMsg.payload.genericPosStatus);

                PointOrient2D<float> p = pm.getPosition();
                outMsg.payload.genericPosStatus.pos.x = p.p.x / 10;
                outMsg.payload.genericPosStatus.pos.y = p.p.y / 10;
                outMsg.payload.genericPosStatus.pos.theta = p.o;
                outMsg.payload.genericPosStatus.date = micros();

                role_send(&outMsg, ROLEMSG_PRIM_POS);
            }
            else {
                prevPos_ms = millis();

                memset(&outMsg, 0, sizeof(outMsg));

                outMsg.header.type = E_GENERIC_POS_STATUS;
                outMsg.header.size = sizeof(outMsg.payload.genericPosStatus);

                PointOrient2D<float> p = pm.getPosition();
                outMsg.payload.genericPosStatus.pos.x = 50;
                outMsg.payload.genericPosStatus.pos.y = 50;
                outMsg.payload.genericPosStatus.pos.theta = 0;
                outMsg.payload.genericPosStatus.date = micros();

                role_send(&outMsg, ROLEMSG_PRIM_POS);
            }
        }

    }
    return 0;
}

//Serial serial;
