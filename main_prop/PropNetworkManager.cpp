/*
 * PropNetworkManager.cpp
 *
 *  Created on: 14 janv. 2016
 *      Author: Sebastien Malissard
 */

#include <PropNetworkManager.h>

#include <assert.h>


// specials bytes
#define START       0xfe // first byte of the message
#define END         0xfd // last byte of the message
#define BACKSLASH   0xfc // add before a data byte if corresponding to a special byte and xor

#define MES_SIZE    8

using namespace std;


PropNetworkManager::PropNetworkManager() : Serial("/dev/ttyACM0", BAUD_9600, CONFIG_8N1), new_pos_available(false), pos_available(false), backslash(false) {
    Open();
}

PropNetworkManager::~PropNetworkManager() {
}


void PropNetworkManager::update() {
    uint8_t byte;

    // get new bytes
    while (1) {
        try {
            Serial::ReadByte(&byte);

            if (backslash) {
                byte ^= BACKSLASH;
                backslash = false;
            }
            else if (byte == BACKSLASH) {
                backslash = true;
                continue;
            }
            else if (byte == START || byte == END) {
                fifo.push({byte, true});
                continue;
            }

            fifo.push({byte, false});
        }
        catch (NotEnoughData&) {
            break;
        }
    }

    cleanBeginnigFIFO();

    // try to find a complete message
    while (fifo.size() >= MES_SIZE) {
        PointOrient2D<float> pos_loc;

        assert(fifo.front().byte == START && fifo.front().special_byte == true);

        fifo.pop();                                                             // delete the START

        byte = fifo.front().byte;
        fifo.pop();                                                             // delete x(1)

        pos_loc.p.x = (int16_t) (byte << 8 | fifo.front().byte);                // get x in mm

        fifo.pop();                                                             // delete x(2)
        byte = fifo.front().byte;
        fifo.pop();                                                             // delete y(1)

        pos_loc.p.y = (int16_t) (byte << 8 | fifo.front().byte);                // get y in mm

        fifo.pop();                                                             // delete y(2)
        byte = fifo.front().byte;
        fifo.pop();                                                             // delete theta(1)

        pos_loc.o = ((float) (byte << 8 | fifo.front().byte)) / 10000;          // get theta in rad

        fifo.pop();                                                             // delete theta(2)

        if (fifo.front().byte == END && fifo.front().special_byte == true) {    // message correct
            fifo.pop();                                                         // delete END

            pos = pos_loc;

            new_pos_available = true;
            pos_available = true;
        }
        // else corrupted data

        // try to find an other message
        cleanBeginnigFIFO();
    }
}

void PropNetworkManager::sendSetPointSpeed(Vector2D<float> v, float theta) {
    queue<float> mes;
    uint8_t byte;

    assert(v.x > -32768 || v.x < 32767);
    assert(v.y > -32768 || v.y < 32767);
    assert(theta > -32.768 || theta < 32.767);

    mes.push(START);
    mes.push((int) v.x >> 8);
    mes.push((int) v.x);
    mes.push((int) v.y >> 8);
    mes.push((int) v.y);
    mes.push(((int) theta * 1000) >> 8);
    mes.push((int)theta * 1000);
    mes.push(END);

    WriteByte(mes.front()); // START
    mes.pop();

    while (mes.size() != 1) {
        byte = mes.front();

        if (byte == START || byte == END || byte == BACKSLASH) {
            WriteByte(BACKSLASH);
            byte ^= BACKSLASH;
        }

        WriteByte(byte);
    }

    WriteByte(mes.front()); // END
    mes.pop();

    assert(mes.empty() == true);
}


bool PropNetworkManager::isNewPositionAvailable() {
    return new_pos_available;
}

bool PropNetworkManager::isPositionAvailable() {
    return pos_available;
}

PointOrient2D<float> PropNetworkManager::getPosition() {
    if (!isPositionAvailable()) {
        throw NoPositionAvailable("getPosition");
    }

    new_pos_available = false;

    return pos;
}


void PropNetworkManager::cleanBeginnigFIFO() {
    if (fifo.empty()) {
        return;
    }

    while (fifo.front().byte != START && fifo.front().special_byte != true) {
        fifo.pop();
        printf("lost data\n");

        if (fifo.empty()) {
            break;
        }
    }
}

PropNetworkManager pnm;
