/*
 * Serial.cpp
 *
 *  Created on: 13 janv. 2016
 *      Author: Sebastien Malissard
 */

#include <Serial.h>

#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdexcept>
#include <iostream>
#include <sys/ioctl.h>
#include <errno.h>

#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)
#define ERR_STR ((std::string)(__FILE__ " line " STRINGIZE(__LINE__) ": "))
#define ERR_STR_ERRNO ((std::string) strerror(errno))
#define ERROR_ERRNO (ERR_STR + ERR_STR_ERRNO)

using namespace std;

Serial::Serial(string device_init, baud baud_init, config config_init) : _baud(baud_init), _config(config_init), device(device_init), fd(-1), is_open(false) {

}

Serial::~Serial() {
}

//http://www.cmrr.umn.edu/~strupp/serial.html#config
void Serial::Open() {

    if (IsOpen()) {
        throw AlreadyOpen(ERR_STR);
    }

    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK ); // open device

    if (fd < 0) {
        throw OpenFailed(ERROR_ERRNO);
    }

    if (tcflush(fd, TCIOFLUSH) < 0) { // flush input and output queue
        throw OpenFailed(ERROR_ERRNO);
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) { // set the serial port in nonblocking mode
        throw OpenFailed(ERROR_ERRNO);
    }

    termios options;
    bzero(&options, sizeof(options)); // clear all the options

    if (cfsetspeed(&options, _baud) < 0) { // set the input and output baud
        throw ConfigurationFailed(ERROR_ERRNO);
    }

    options.c_iflag = IGNBRK;           // ignore break
    options.c_oflag = 0;                // raw output
    options.c_cflag = CLOCAL | CREAD;   // do not change owner of port and enable receiver
    options.c_lflag = 0;                // raw input

    switch (_config) {
        case CONFIG_8N1:
            options.c_cflag |= CS8;
            break;

        case CONFIG_7E1:
            options.c_cflag |= PARENB | CS7;
            break;

        case CONFIG_7O1:
            options.c_cflag |= PARENB | PARODD | CS7;
            break;
    }

    if (tcsetattr(fd, TCSANOW, &options) < 0 ) { // set the new options for the serial port
        throw ConfigurationFailed(ERROR_ERRNO);
    }

    is_open = true;
}

unsigned int Serial::DataAvailable() {

    if (!IsOpen()) {
        throw NotOpen(ERR_STR);
    }

    int nb_bytes = 0;

    if (ioctl(fd, FIONREAD, &nb_bytes) < 0) {
        // TODO
    }

    return nb_bytes;
}

void Serial::ReadByte(uint8_t *byte) {
    Read(byte, 1);
}

void Serial::Read(uint8_t *buf, unsigned int nb_bytes) {

    if (!IsOpen()) {
        throw NotOpen(ERR_STR);
    }

    if (DataAvailable() < nb_bytes) {
        throw NotEnoughData(ERR_STR);
    }

    if (read(fd, buf, nb_bytes) < 0) {
        throw runtime_error(ERROR_ERRNO);
    }
}

void Serial::WriteByte(uint8_t byte) {
    Write(&byte, 1);
}

void Serial::Write(uint8_t *buf, unsigned int nb_bytes) {

    if (!IsOpen()) {
        throw NotOpen(ERR_STR);
    }

    if (write(fd, buf, nb_bytes) < 0) {
        throw runtime_error(ERROR_ERRNO);
    }
}

bool Serial::IsOpen() {
    return is_open;
}



//extern Serial serial;
