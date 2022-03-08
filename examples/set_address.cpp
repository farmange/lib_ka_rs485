#include "ka_rs485.h"
#include "logger.h"

#include <getopt.h>
#include <unistd.h>
// #include <iostream>
// #include <sstream>

using namespace KinovaApi;

/* Program to communicate with usb to serial bridge */
void print_usage()
{
    printf("Usage: ka_rs485_set_address -a addr -n new_addr\n");
}

int main(int argc, char *argv[])
{
    int opt = 0;
    uint16_t device_addr = 0;
    uint16_t new_device_addr = 0;

    int eth_flag = 0;

    Logger::logging_level_t log_level = Logger::INFO;
    // Specifying the expected options
    static struct option long_options[] = {
        {"addr", required_argument, 0, 'a'},
        {"new_addr", required_argument, 0, 'n'},
        {"verbose", required_argument, 0, 'v'},
        {"ethernet", no_argument, &eth_flag, 1},
        {0, 0, 0, 0}};

    int long_index = 0;
    while ((opt = getopt_long(argc, argv, "a:n:v:",
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
        case 'n':
            new_device_addr = (uint16_t)(atoi(optarg));
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

    if (device_addr == 0 || new_device_addr == 0)
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
        LOG_ERROR_STREAM("Device Initialisation error: " << status);
        return 0;
    }

    status = api.setAddress(device_addr, new_device_addr, position);
    if (status == KaRS485Api::API_OK)
    {
        LOG_INFO_STREAM("Set address done");
    }
    else
    {
        LOG_ERROR_STREAM("Set address error : " << status);
        return 0;
    }
    return 0;
}