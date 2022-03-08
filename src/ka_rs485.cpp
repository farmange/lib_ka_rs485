#include "ka_rs485.h"
#include "logger.h"

#include <arpa/inet.h>
#include <dlfcn.h>
#include <unistd.h>
#include <string.h>
// #include <iostream>
// #include <sstream>
// #include <sys/ioctl.h>
// #include <linux/serial.h>
using namespace std;

#define RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES_REPLY 2

#define RS485_MSG_GET_ACTUALPOSITION_REPLY 1
#define RS485_MSG_FEEDTHROUGH_REPLY 1
#define RS485_MSG_GET_POSITION_COMMAND_REPLY 1
#define RS485_MSG_GET_DEVICE_INFO_REPLY 1
#define RS485_MSG_GET_CODE_VERSION_REPLY 1
#define RS485_MSG_GET_TEMPERATURE_REPLY 1
#define RS485_MSG_SET_TEMPERATURE_REPLY 1
#define RS485_GET_ENCODER_STATUSSES_REPLY 1
#define RS485_MSG_SET_ADDRESS_REPLY 1
#define RS485_MSG_CLEAR_FAULT_FLAG_REPLY 1
#define RS485_MSG_STAR_ASSERV_REPLY 1
#define RS485_MSG_STOP_ASSERV_REPLY 1
#define RS485_MSG_POSITION_MAX_MIN_REPLY 1
#define RS485_MSG_KP_GAIN_REPLY 1
#define RS485_MSG_KI_KD_GAIN_REPLY 1
#define RS485_MSG_PROGRAM_JOINT_ZERO_REPLY 1
#define RS485_SET_PID_FILTERS_REPLY 1
#define RS485_SET_ZERO_TORQUESENSOR_REPLY 1
#define RS485_SET_GAIN_TORQUESENSOR_REPLY 1
#define RS485_SET_CONTROL_WITH_ENCODER_REPLY 1
#define RS485_SET_PID_ADVANCED_PARAMETERS_REPLY 1
#define RS485_MSG_SPEED_ACCEL_MAX_REPLY 1
#define RS485_MSG_CURRENT_TORQUE_MAX_REPLY 1

#define GET_EXPECTED_REPLY(command) command##_REPLY

namespace KinovaApi
{
    KaRS485Api::KaRS485Api()
    {
    }

    KaRS485Api::ApiStatus_t KaRS485Api::init(const bool &debug_log, const bool &enable_ethernet)
    {
        api_mutex_.lock();

        LOG_INFO_STREAM("Initialize KaRS485Api instance... (debug logging : "
                        << std::boolalpha << debug_log << ", ethernet : "
                        << std::boolalpha << enable_ethernet << ") ...");
        if (debug_log == false)
        {
            Logger::instance().setLevel(Logger::ERROR);
        }

        LOG_INFO_STREAM("Load kinova shared library...");
        // We load the API.
        if (enable_ethernet)
        {
            comm_handler = dlopen("/opt/JACO-SDK/API/Kinova.API.EthCommLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
        }
        else
        {
            comm_handler = dlopen("Kinova.API.CommLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
        }

        if (comm_handler == NULL)
        {
            LOG_ERROR_STREAM("Failed to load library with dlopen : " << dlerror());
            return API_INIT_ERROR;
        }
        fcn_ptr_init_eth = nullptr;
        fcn_ptr_init = nullptr;
        fcn_ptr_activate = nullptr;
        fcn_ptr_read = nullptr;
        fcn_ptr_write = nullptr;
        fcn_ptr_close = nullptr;
        // Initialization of the fucntion pointers.
        if (enable_ethernet)
        {
            fcn_ptr_init_eth = (int (*)(EthernetCommConfig & config)) dlsym(comm_handler, "Ethernet_Communication_InitCommunicationEthernet");
            fcn_ptr_activate = (int (*)())dlsym(comm_handler, "Ethernet_Communication_OpenRS485_Activate");
            fcn_ptr_read = (int (*)(RS485_Message * PackagesIn, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler, "Ethernet_Communication_OpenRS485_Read");
            fcn_ptr_write = (int (*)(RS485_Message * PackagesOut, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler, "Ethernet_Communication_OpenRS485_Write");
            fcn_ptr_close = (int (*)())dlsym(comm_handler, "Ethernet_Communication_CloseCommunication");
        }
        else
        {
            fcn_ptr_init = (int (*)())dlsym(comm_handler, "InitCommunication");
            fcn_ptr_activate = (int (*)())dlsym(comm_handler, "OpenRS485_Activate");
            fcn_ptr_read = (int (*)(RS485_Message * PackagesIn, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler, "OpenRS485_Read");
            fcn_ptr_write = (int (*)(RS485_Message * PackagesOut, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler, "OpenRS485_Write");
            fcn_ptr_close = (int (*)())dlsym(comm_handler, "CloseCommunication");
        }

        ethernet_conf.localBcastPort = 25015;
        ethernet_conf.localCmdport = 25025;
        ethernet_conf.robotPort = 55000;
        ethernet_conf.localIpAddress = inet_addr("192.168.100.100");
        ethernet_conf.robotIpAddress = inet_addr("192.168.100.10");
        ethernet_conf.subnetMask = inet_addr("255.255.255.0");
        ethernet_conf.rxTimeOutInMs = 1000;

        // Initialization of the API
        LOG_INFO("Initialization of the API...");
        if (((enable_ethernet && fcn_ptr_init_eth != NULL) || fcn_ptr_init != NULL) && fcn_ptr_activate != NULL && fcn_ptr_read != NULL && fcn_ptr_write != NULL)
        {
            int result;
            if (enable_ethernet)
            {
                result = fcn_ptr_init_eth(ethernet_conf);
            }
            else
            {
                result = fcn_ptr_init();
            }

            if (result != NO_ERROR_KINOVA)
            {
                LOG_ERROR_STREAM("Initialization communication error : " << result);
                return API_INIT_ERROR;
            }
            // We activate the RS-485 comm API. From here you cannot control the robot with the Joystick or
            // with the normal USB function. Only RS-485 command will be accepted. Reboot the robot to get
            // back to normal control.
            fcn_ptr_activate();
        }
        else
        {
            LOG_ERROR_STREAM("Not all function pointer are initialized !");
            return API_INIT_ERROR;
        }
        LOG_INFO_STREAM("Done");

        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t KaRS485Api::readWrite_(CommLayer::message_t *writeMessage, CommLayer::message_t *readMessage, const int &expectedResponseMsg)
    {
        int nb_msg_sent = 0;
        int nb_msg_read = 0;

        for (int i = 0; i < MAX_WRITE_RETRY; i++)
        {
            bool transmission_ok = false;
            // comm_.commLayerWrite(writeMessage, 1, nb_msg_sent);
            LOG_DEBUG_STREAM("Write RS485 message...");
            fcn_ptr_write((RS485_Message *)writeMessage, 1, nb_msg_sent);
            LOG_DEBUG_STREAM("Done");

            if (nb_msg_sent != 1)
            {
                /* Try to send message again */
                usleep(20);
                LOG_DEBUG_STREAM("Fail to write message (nb_msg_sent=" << nb_msg_sent << ")");
                continue;
            }

            usleep(50);

            for (int j = 0; j < MAX_READ_RETRY; j++)
            {
                RS485_Message dummy_rcv_msg[50] = {};
                fcn_ptr_read(dummy_rcv_msg, expectedResponseMsg, nb_msg_read);
                if (nb_msg_read != expectedResponseMsg)
                {
                    /* Try to receive message again */
                    usleep(2);
                    LOG_DEBUG_STREAM("Fail to read message (nb_msg_read=" << nb_msg_read << ", expected : " << expectedResponseMsg << ")");
                    continue;
                }
                memcpy(readMessage, &dummy_rcv_msg, expectedResponseMsg * sizeof(RS485_Message));
                LOG_DEBUG_STREAM("Message successfully write and reply read");
                transmission_ok = true;
                break;
            }
            if (transmission_ok)
            {
                break;
            }
        }
        if (nb_msg_sent != 1)
        {
            return API_WRITE_ERROR;
        }
        if (nb_msg_read != expectedResponseMsg)
        {
            return API_READ_ERROR;
        }

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::deviceInitialisation(const uint16_t &jointAddress, float &jointPosition)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_SET_ADDRESS;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = 0;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, GET_EXPECTED_REPLY(RS485_MSG_SET_ADDRESS));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointPosition = msgRead.DataFloat[1];
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::clearError(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_CLEAR_FAULT_FLAG;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = 0x00;
        msgWrite.DataLong[1] = 0x00;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, GET_EXPECTED_REPLY(RS485_MSG_CLEAR_FAULT_FLAG));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::startMotorControl(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_STAR_ASSERV;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = RS485_MSG_STAR_ASSERV;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, GET_EXPECTED_REPLY(RS485_MSG_STAR_ASSERV));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::stopMotorControl(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_STOP_ASSERV;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = RS485_MSG_STOP_ASSERV;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, GET_EXPECTED_REPLY(RS485_MSG_STOP_ASSERV));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::getActualPosition(const uint16_t &jointAddress, float &jointCurrent, float &jointPositionHall, float &jointSpeed, float &jointTorque)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_GET_ACTUALPOSITION;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = 0;
        msgWrite.DataLong[1] = 0;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, GET_EXPECTED_REPLY(RS485_MSG_GET_ACTUALPOSITION));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead.DataFloat[0];
        jointPositionHall = msgRead.DataFloat[1];
        jointSpeed = msgRead.DataFloat[2];
        jointTorque = msgRead.DataFloat[3];
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::setCommandAllValue(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                   float &jointSpeed, float &jointTorque, float &jointPMW, float &jointPositionOptical,
                                   short &jointAccelX, short &jointAccelY, short &jointAccelZ, short &jointTemp)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead[2];
        msgWrite.Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataFloat[0] = jointCommand;
        msgWrite.DataFloat[1] = jointCommand;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead[0], GET_EXPECTED_REPLY(RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead[0].DataFloat[0];
        jointPositionHall = msgRead[0].DataFloat[1];
        jointSpeed = msgRead[0].DataFloat[2];
        jointTorque = msgRead[0].DataFloat[3];

        jointPMW = msgRead[1].DataFloat[0];
        jointPositionOptical = msgRead[1].DataFloat[1];
        jointAccelX = msgRead[1].DataShort[4];
        jointAccelY = msgRead[1].DataShort[5];
        jointAccelZ = msgRead[1].DataShort[6];
        jointTemp = msgRead[1].DataShort[7];
        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t
    KaRS485Api::setPositionCommand(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                   float &jointSpeed, float &jointTorque)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead[2];
        msgWrite.Command = RS485_MSG_GET_POSITION_COMMAND;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataFloat[0] = jointCommand;
        msgWrite.DataFloat[1] = jointCommand;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead[0], GET_EXPECTED_REPLY(RS485_MSG_GET_POSITION_COMMAND));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead[0].DataFloat[0];
        jointPositionHall = msgRead[0].DataFloat[1];
        jointSpeed = msgRead[0].DataFloat[2];
        jointTorque = msgRead[0].DataFloat[3];

        api_mutex_.unlock();

        return API_OK;
    }

    KaRS485Api::ApiStatus_t KaRS485Api::getPosition()
    {
        api_mutex_.lock();
        api_mutex_.unlock();

        return API_OK;
    }

} // namespace KinovaApi

// a statically declared instance of our derived factory class
// ApiFactory Factory;
