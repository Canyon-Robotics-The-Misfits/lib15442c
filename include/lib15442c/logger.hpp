#pragma once

#ifndef LIB15442C_MOCK_DEVICES_ONLY

#include "pros/misc.h"
#define GET_TIME pros::c::millis()

#else

#define GET_TIME 0

#endif

#include <math.h>

#define TIME_SECONDS (int)(round(GET_TIME / 1000.0))
#define LOG(mode, format, ...) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60, __VA_ARGS__)
#define LOG_TEXT(mode, format) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60)

// log level debug
#define DEBUG(format, ...) LOG("DEBUG", format, __VA_ARGS__)
// log level info
#define INFO(format, ...) LOG("\033[34mINFO\033[0m", format, __VA_ARGS__)
// log level warn
#define WARN(format, ...) LOG("\033[33mWARN\033[0m", format, __VA_ARGS__)
// log level error
#define ERROR(format, ...) LOG("\033[31mERROR\033[0m", format, __VA_ARGS__)

// log level debug with text only
#define DEBUG_TEXT(format) LOG_TEXT("DEBUG", format)
// log level info with text only
#define INFO_TEXT(format) LOG_TEXT("\033[34mINFO\033[0m", format)
// log level warn with text only
#define WARN_TEXT(format) LOG_TEXT("\033[33mWARN\033[0m", format)
// log level error with text only
#define ERROR_TEXT(format) LOG_TEXT("\033[31mERROR\033[0m", format)