#include "ka_rs485.h"
#include "logger.h"

#include <getopt.h>
#include <unistd.h>
// #include <iostream>
// #include <sstream>

using namespace KinovaApi;

const int kLoopIterationNumber = 1000;

/* Program to communicate with usb to serial bridge */
void print_usage()
{
    printf("Usage: ka_rs485_test -a addr\n");
}

int main(int argc, char *argv[])
{
    int opt = 0;
    uint16_t device_addr = 0;
    int eth_flag = 0;

    Logger::logging_level_t log_level = Logger::INFO;
    // Specifying the expected options
    static struct option long_options[] = {
        {"addr", required_argument, 0, 'a'},
        {"verbose", required_argument, 0, 'v'},
        {"ethernet", no_argument, &eth_flag, 1},
        {0, 0, 0, 0}};

    int long_index = 0;
    while ((opt = getopt_long(argc, argv, "a:v:",
                              long_options, &long_index)) != -1)
    {

        switch (opt)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[long_index].flag != 0)
                break;
            printf("option %s", long_options[long_index].name);
            if (optarg)
                printf(" with arg %s", optarg);
            printf("\n");
            break;

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
    LOG_INFO_STREAM("eth_flag: " << eth_flag);
    KaRS485Api api;
    if (eth_flag)
    {
        api.init(true, true);
    }
    else
    {
        api.init(true, false);
    }

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

    LOG_WARN_STREAM("=======================");
    LOG_WARN_STREAM("setCommandAllValue");
    LOG_WARN_STREAM(" -> ethernet : " << std::boolalpha << (bool)eth_flag);
    LOG_WARN_STREAM(" -> nb iteration : " << kLoopIterationNumber);
    LOG_WARN_STREAM("=======================");

    double elaspe_time_mean;
    double elaspe_time_min;
    double elaspe_time_max;
    int error_counter;

    elaspe_time_mean = 0.0;
    elaspe_time_min = 10000.0;
    elaspe_time_max = 0.0;
    error_counter = 0;
    KinovaApi::KaRS485Api::ApiStatus_t ret;
    while (count < kLoopIterationNumber)
    {
        jointCommand += 30 * (0.0025);

        auto start_chrono = std::chrono::system_clock::now();
        ret = api.setCommandAllValue(jointAddress, jointCommand, jointCurrent, jointPositionHall,
                                     jointSpeed, jointTorque, jointPMW, jointPositionOptical,
                                     jointAccelX, jointAccelY, jointAccelZ, jointTemp);
        auto end_chrono = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_chrono - start_chrono;
        auto start_e = std::chrono::duration_cast<std::chrono::milliseconds>(start_chrono.time_since_epoch()).count();
        auto end_e = std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono.time_since_epoch()).count();

        LOG_INFO_STREAM("start: " << start_e
                                  << " | end: " << end_e
                                  << " | elapsed time: " << elapsed_seconds.count() * 1000.);

        elaspe_time_mean += (elapsed_seconds.count() * 1000.);
        elaspe_time_max = std::max(elaspe_time_max, elapsed_seconds.count() * 1000.);
        elaspe_time_min = std::min(elaspe_time_min, elapsed_seconds.count() * 1000.);

        if (ret != KinovaApi::KaRS485Api::ApiStatus_t::API_OK)
        {
            error_counter++;
        }
        LOG_DEBUG("---");
        LOG_DEBUG_STREAM("jointAddress :" << jointAddress);
        LOG_DEBUG_STREAM("jointCommand :" << jointCommand);
        LOG_DEBUG_STREAM("jointCurrent :" << jointCurrent);
        LOG_DEBUG_STREAM("jointPositionHall :" << jointPositionHall);
        LOG_DEBUG_STREAM("jointSpeed :" << jointSpeed);
        LOG_DEBUG_STREAM("jointTorque :" << jointTorque);
        LOG_DEBUG_STREAM("jointPMW :" << jointPMW);
        LOG_DEBUG_STREAM("jointPositionOptical :" << jointPositionOptical);
        LOG_DEBUG_STREAM("jointAccelX :" << jointAccelX);
        LOG_DEBUG_STREAM("jointAccelY :" << jointAccelY);
        LOG_DEBUG_STREAM("jointAccelZ :" << jointAccelZ);
        LOG_DEBUG_STREAM("jointTemp :" << jointTemp);
        usleep(5000);
        count++;
    }
    LOG_WARN_STREAM("=> elaspe_time_mean: " << elaspe_time_mean / kLoopIterationNumber);
    LOG_WARN_STREAM("=> elaspe_time_max: " << elaspe_time_max);
    LOG_WARN_STREAM("=> elaspe_time_min: " << elaspe_time_min);
    LOG_WARN_STREAM("=> error_counter: " << error_counter);

    LOG_WARN_STREAM("=======================");
    LOG_WARN_STREAM("SetPositionCommand ");
    LOG_WARN_STREAM(" -> ethernet : " << std::boolalpha << (bool)eth_flag);
    LOG_WARN_STREAM(" -> nb iteration : " << kLoopIterationNumber);
    LOG_WARN_STREAM("=======================");

    count = 0;
    elaspe_time_mean = 0.0;
    elaspe_time_min = 10000.0;
    elaspe_time_max = 0.0;
    error_counter = 0;
    while (count < kLoopIterationNumber)
    {
        jointCommand -= 30 * (0.0025);

        auto start_chrono = std::chrono::system_clock::now();
        api.setPositionCommand(jointAddress, jointCommand, jointCurrent, jointPositionHall,
                               jointSpeed, jointTorque);
        auto end_chrono = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_chrono - start_chrono;
        auto start_e = std::chrono::duration_cast<std::chrono::milliseconds>(start_chrono.time_since_epoch()).count();
        auto end_e = std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono.time_since_epoch()).count();

        LOG_INFO_STREAM("start: " << start_e
                                  << " | end: " << end_e
                                  << " | elapsed time: " << elapsed_seconds.count() * 1000.);
        elaspe_time_mean += (elapsed_seconds.count() * 1000.);
        elaspe_time_max = std::max(elaspe_time_max, elapsed_seconds.count() * 1000.);
        elaspe_time_min = std::min(elaspe_time_min, elapsed_seconds.count() * 1000.);

        if (ret != KinovaApi::KaRS485Api::ApiStatus_t::API_OK)
        {
            error_counter++;
        }

        LOG_DEBUG("---");
        LOG_DEBUG_STREAM("jointAddress :" << jointAddress);
        LOG_DEBUG_STREAM("jointCommand :" << jointCommand);
        LOG_DEBUG_STREAM("jointCurrent :" << jointCurrent);
        LOG_DEBUG_STREAM("jointPositionHall :" << jointPositionHall);
        LOG_DEBUG_STREAM("jointSpeed :" << jointSpeed);
        LOG_DEBUG_STREAM("jointTorque :" << jointTorque);
        usleep(5000);
        count++;
    }

    LOG_WARN_STREAM("=> elaspe_time_mean: " << elaspe_time_mean / kLoopIterationNumber);
    LOG_WARN_STREAM("=> elaspe_time_max: " << elaspe_time_max);
    LOG_WARN_STREAM("=> elaspe_time_min: " << elaspe_time_min);
    LOG_WARN_STREAM("=> error_counter: " << error_counter);

    return 0;
}