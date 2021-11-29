#include "ka_rs485.h"
#include "logger.h"

#include <unistd.h>
#include <getopt.h>
// #include <iostream>
// #include <sstream>

using namespace KinovaApi;

/* Program to communicate with usb to serial bridge */
void print_usage()
{
    printf("Usage: ka_rs485_test -a addr\n");
}

int main(int argc, char *argv[])
{
    int opt = 0;
    uint16_t device_addr = 0;

    Logger::logging_level_t log_level = Logger::INFO;
    //Specifying the expected options
    //The two options l and b expect numbers as argument
    static struct option long_options[] = {
        {"addr", required_argument, 0, 'a'},
        {"verbose", required_argument, 0, 'v'},
        {0, 0, 0, 0}};

    int long_index = 0;
    while ((opt = getopt_long(argc, argv, "a:v:",
                              long_options, &long_index)) != -1)
    {

        switch (opt)
        {
        case 'a':
            device_addr = (uint16_t)(atoi(optarg));
            break;
        case 'v':
            log_level = Logger::logging_level_t(atoi(optarg));
            break;
        default:
            print_usage();
            exit(EXIT_FAILURE);
        }
    }
    Logger::instance().setLevel(log_level);

    if (device_addr == 0)
    {
        print_usage();
        exit(EXIT_FAILURE);
    }

    KaRS485Api api;
    api.init(true);
    float position;
    KaRS485Api::ApiStatus_t status = api.deviceInitialisation(device_addr, position);
    if (status == KaRS485Api::API_OK)
    {
        LOG_INFO_STREAM("Device Initialisation position: " << position);
    }
    else
    {
        LOG_INFO_STREAM("Device Initialisation error: " << status);
        return 0;
    }

    uint16_t jointAddress = device_addr;
    float jointCommand = position;

    float jointCurrent = 0;
    float jointPositionHall = 0;
    float jointSpeed = 0;
    float jointTorque = 0;
    float jointPMW = 0;
    float jointPositionOptical = 0;
    short jointAccelX = 0;
    short jointAccelY = 0;
    short jointAccelZ = 0;
    short jointTemp = 0;
    int count = 0;
    while (count < 100)
    {
        jointCommand += 60 * (0.0025);

        api.setCommandAllValue(jointAddress, jointCommand, jointCurrent, jointPositionHall,
                               jointSpeed, jointTorque, jointPMW, jointPositionOptical,
                               jointAccelX, jointAccelY, jointAccelZ, jointTemp);
        LOG_INFO("---");
        LOG_INFO_STREAM("jointAddress :" << jointAddress);
        LOG_INFO_STREAM("jointCommand :" << jointCommand);
        LOG_INFO_STREAM("jointCurrent :" << jointCurrent);
        LOG_INFO_STREAM("jointPositionHall :" << jointPositionHall);
        LOG_INFO_STREAM("jointSpeed :" << jointSpeed);
        LOG_INFO_STREAM("jointTorque :" << jointTorque);
        LOG_INFO_STREAM("jointPMW :" << jointPMW);
        LOG_INFO_STREAM("jointPositionOptical :" << jointPositionOptical);
        LOG_INFO_STREAM("jointAccelX :" << jointAccelX);
        LOG_INFO_STREAM("jointAccelY :" << jointAccelY);
        LOG_INFO_STREAM("jointAccelZ :" << jointAccelZ);
        LOG_INFO_STREAM("jointTemp :" << jointTemp);
        usleep(9000);
        count++;
    }

    return 0;
}