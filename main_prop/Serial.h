/*
 * Serial.h
 *
 *  Created on: 13 janv. 2016
 *      Author: Sebastien Malissard
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <string>
#include <termios.h>
#include <stdexcept>
#include <unistd.h>

class Serial {
    public:
        typedef enum {
            BAUD_50      = B50,
            BAUD_75      = B75,
            BAUD_110     = B110,
            BAUD_134     = B134,
            BAUD_150     = B150,
            BAUD_200     = B200,
            BAUD_300     = B300,
            BAUD_600     = B600,
            BAUD_1200    = B1200,
            BAUD_1800    = B1800,
            BAUD_2400    = B2400,
            BAUD_4800    = B4800,
            BAUD_9600    = B9600,
            BAUD_19200   = B19200,
            BAUD_38400   = B38400,
            BAUD_57600   = B57600,
            BAUD_115200  = B115200,
            BAUD_230400  = B230400,
            BAUD_460800  = B460800,
            BAUD_500000  = B500000,
            BAUD_576000  = B576000,
            BAUD_921600  = B921600,
            BAUD_1000000 = B1000000,
            BAUD_1152000 = B1152000,
            BAUD_1500000 = B1500000,
            BAUD_2000000 = B2000000,
            BAUD_2500000 = B2500000,
            BAUD_3000000 = B3000000,
            BAUD_3500000 = B3500000,
            BAUD_4000000 = B4000000,
        } baud;

        typedef enum {
            CONFIG_8N1,
            CONFIG_7E1,
            CONFIG_7O1
        } config;


        Serial(std::string device_init, baud baud_init, config config_init);
        ~Serial();

        void Open();
        void ReadByte(uint8_t *byte);
        void Read(uint8_t *buf, unsigned int nb_bytes);
        void WriteByte(uint8_t byte);
        void Write(uint8_t *buf, unsigned int nb_bytes);

        unsigned int DataAvailable();
        bool IsOpen();

    class OpenFailed : public std::runtime_error {
        public:
            OpenFailed(const std::string& whatArg) : runtime_error("OpenFailed : " + whatArg) {
            }
        };

    class AlreadyOpen : public std::logic_error {
        public:
            AlreadyOpen(const std::string& whatArg) : logic_error("AlreadyOpen : " + whatArg) {
            }
        };

    class ConfigurationFailed : public std::runtime_error {
        public:
            ConfigurationFailed(const std::string& whatArg) : runtime_error("ConfigurationFailed : " + whatArg) {
            }
        };

    class NotOpen : public std::logic_error {
        public:
            NotOpen(const std::string& whatArg) : logic_error("NotOpen : " + whatArg) { }
        };

    class NotEnoughData : public std::runtime_error {
        public:
            NotEnoughData(const std::string& whatArg) : runtime_error("NotEnoughData : " + whatArg) { }
        };


    private:
        baud _baud;
        config _config;
        std::string device;
        int fd;
        bool is_open;
};

#endif /* SERIAL_H_ */
