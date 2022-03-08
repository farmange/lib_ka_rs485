#ifndef _KA_RS485_H
#define _KA_RS485_H

// // C library headers
// #include <stdio.h>
// #include <string.h>

// // C++ library headers
// #include <string>

// // Linux headers
// #include <fcntl.h>   // Contains file controls like O_RDWR
// #include <errno.h>   // Error integer and strerror() function
// #include <termios.h> // Contains POSIX terminal control definitions
// #include <unistd.h>  // write(), read(), close()
// #include "Kinova.API.EthCommLayerUbuntu.h"
//#include "Kinova.API.USBCommLayerUbuntu.h"
#include "Kinova.API.EthCommLayerUbuntu.h"
#include "Kinova.API.CommLayerUbuntu.h"

#include <mutex>

#define MAX_READ_RETRY 10
#define MAX_WRITE_RETRY 10

namespace KinovaApi
{
    class CommLayer
    {
    public:
        // This structure represents a RS-485 message
        typedef struct message_s
        {
            // Command ID of the message. Use #define from the COMMAND ID LIST above.
            short Command;

            // Source of the message. If this is an actuator, it will be an address.
            unsigned char SourceAddress;

            // Destination of the message. Use the address of the actuator.
            unsigned char DestinationAddress;

            // Data of the message displayed as unsigned char, float or unsigned long.
            union
            {
                unsigned char DataByte[16];
                uint16_t DataShort[8];
                float DataFloat[4];
                unsigned int DataLong[4];
            };
        } message_t;
    };

    class KaRS485Api
    {
    public:
        typedef enum ApiStatus_s
        {
            API_OK = 0,
            API_INIT_ERROR,
            API_WRITE_ERROR,
            API_READ_ERROR
        } ApiStatus_t;

    public:
        KaRS485Api();
        virtual ~KaRS485Api(){};
        ApiStatus_t init(const bool &debug_log = false, const bool &enable_ethernet = false);
        ApiStatus_t deviceInitialisation(const uint16_t &jointAddress, float &jointPosition);
        ApiStatus_t setAddress(const uint16_t &jointAddress, const uint16_t &newJointAddress, float &jointPosition);
        ApiStatus_t clearError(const uint16_t &jointAddress);
        ApiStatus_t startMotorControl(const uint16_t &jointAddress);
        ApiStatus_t stopMotorControl(const uint16_t &jointAddress);
        ApiStatus_t getActualPosition(const uint16_t &jointAddress, float &jointCurrent, float &jointPositionHall, float &jointSpeed, float &jointTorque);
        ApiStatus_t setCommandAllValue(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                       float &jointSpeed, float &jointTorque, float &jointPMW, float &jointPositionOptical,
                                       short &jointAccelX, short &jointAccelY, short &jointAccelZ, short &jointTemp);
        ApiStatus_t setPositionCommand(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent,
                                       float &jointPositionHall, float &jointSpeed, float &jointTorque);
        ApiStatus_t getPosition();

    private:
        ApiStatus_t readWrite_(CommLayer::message_t *writeMessage, CommLayer::message_t *readMessage, const int &expectedResponseMsg);
        std::mutex api_mutex_;

        EthernetCommConfig ethernet_conf;

        // A handle needed to open the API(library).
        void *comm_handler;

        // Function pointers to access API's function.
        int (*fcn_ptr_init)();
        int (*fcn_ptr_init_eth)(EthernetCommConfig &config);
        int (*fcn_ptr_activate)(); // FUNCTION TO ACTIVATE USB - RS485 MODE //
        int (*fcn_ptr_read)(RS485_Message *PackagesIn, int QuantityWanted, int &ReceivedQtyIn);
        int (*fcn_ptr_write)(RS485_Message *PackagesOut, int QuantityWanted, int &ReceivedQtyIn);
        int (*fcn_ptr_close)();
    };

} // namespace KinovaApi

// // the class of the plugin factory
// #include <iostream>

// class ApiFactory //: factory
// {
// public:
//     KinovaApi::APILayer *makedyn()
//     {
//         std::cout << "    making foo [foo.cc ApiFactory::makedyn()]" << std::endl;
//         return new KinovaApi::APILayer;
//     }
// };

#endif