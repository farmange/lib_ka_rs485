#include "ka_rs485.h"
#include "logger.h"

#include <unistd.h>
#include <dlfcn.h>

// #include <iostream>
// #include <sstream>
// #include <sys/ioctl.h>
// #include <linux/serial.h>
using namespace std;

namespace KinovaApi
{
    KaRS485Api::KaRS485Api()
    {
    }

    KaRS485Api::ApiStatus_t KaRS485Api::init(const bool &debug_log)
    {
        api_mutex_.lock();

        LOG_INFO_STREAM("Initialize KaRS485Api instance... (debug logging : " << std::boolalpha << debug_log << ") ...");
        if (debug_log == false)
        {
            Logger::instance().setLevel(Logger::ERROR);
        }

        LOG_INFO_STREAM("Load kinova shared library...");
        //We load the API.
        comm_handler = dlopen("Kinova.API.CommLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);
        if (comm_handler == NULL)
        {
            LOG_ERROR_STREAM("Failed to load library with dlopen : " << dlerror() );
            return API_INIT_ERROR;
        }

        //Initialization of the fucntion pointers.
        fcn_ptr_init = (int (*)()) dlsym(comm_handler,"InitCommunication");
        fcn_ptr_activate = (int (*)()) dlsym(comm_handler,"OpenRS485_Activate");
        fcn_ptr_read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler,"OpenRS485_Read");
        fcn_ptr_write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn)) dlsym(comm_handler,"OpenRS485_Write");
        fcn_ptr_close = (int(*)()) dlsym(comm_handler, "CloseCommunication");

        if(fcn_ptr_init != NULL && fcn_ptr_activate != NULL && fcn_ptr_read != NULL && fcn_ptr_write != NULL)
        {
            // Initialization of the API
            LOG_INFO("Initialization of the API...");
            int result = fcn_ptr_init();

            if(result != NO_ERROR_KINOVA)
            {
                LOG_ERROR_STREAM("Initialization communication error : " << result);
                return API_INIT_ERROR;
            }
            //We activate the RS-485 comm API. From here you cannot control the robot with the Joystick or
            //with the normal USB function. Only RS-485 command will be accepted. Reboot the robot to get
            //back to normal control.
            fcn_ptr_activate();
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
            fcn_ptr_write((RS485_Message*)writeMessage, 1, nb_msg_sent);            
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
                // comm_.commLayerRead(readMessage, expectedResponseMsg, nb_msg_read);
                fcn_ptr_read((RS485_Message*)readMessage, expectedResponseMsg, nb_msg_read);
                if (nb_msg_read != expectedResponseMsg)
                {
                    /* Try to receive message again */
                    usleep(2);
                    LOG_DEBUG_STREAM("Fail to read message (nb_msg_read=" << nb_msg_read << ")");
                    continue;
                }

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

    const int KaRS485Api::getExpectedReply_(uint16_t command)
    {
        switch (command)
        {
        case RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES:
            return 2;
            break;

        case RS485_MSG_GET_ACTUALPOSITION:
        case RS485_MSG_FEEDTHROUGH:
        case RS485_MSG_GET_POSITION_COMMAND:
        case RS485_MSG_GET_DEVICE_INFO:
        case RS485_MSG_GET_CODE_VERSION:
        case RS485_MSG_GET_TEMPERATURE:
        case RS485_MSG_SET_TEMPERATURE:
        case RS485_GET_ENCODER_STATUSSES:
        case RS485_MSG_SET_ADDRESS:
        case RS485_MSG_CLEAR_FAULT_FLAG:
        case RS485_MSG_STAR_ASSERV:
        case RS485_MSG_STOP_ASSERV:
        case RS485_MSG_POSITION_MAX_MIN:
        case RS485_MSG_KP_GAIN:
        case RS485_MSG_KI_KD_GAIN:
        case RS485_MSG_PROGRAM_JOINT_ZERO:
        case RS485_SET_PID_FILTERS:
        case RS485_SET_ZERO_TORQUESENSOR:
        case RS485_SET_GAIN_TORQUESENSOR:
        case RS485_SET_CONTROL_WITH_ENCODER:
        case RS485_SET_PID_ADVANCED_PARAMETERS:
        case RS485_MSG_SPEED_ACCEL_MAX:
        case RS485_MSG_CURRENT_TORQUE_MAX:
            return 1;
            break;

        default:
            return 0;
            break;
        }
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

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_SET_ADDRESS));
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

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_CLEAR_FAULT_FLAG));
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

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_STAR_ASSERV));
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

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_STOP_ASSERV));
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

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_GET_ACTUALPOSITION));
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

        status = readWrite_(&msgWrite, &msgRead[0], getExpectedReply_(RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES));
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

        status = readWrite_(&msgWrite, &msgRead[0], getExpectedReply_(RS485_MSG_GET_POSITION_COMMAND));
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
