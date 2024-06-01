#pragma once

#ifndef LIB15442C_MOCK_DEVICES_ONLY

#include "pros/misc.h"
#define GET_TIME pros::c::millis()

#else

#define GET_TIME 0

#endif

#include <math.h>

#define TIME_SECONDS (int)(round(GET_TIME / 1000))
#define LOG(mode, format, ...) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60, __VA_ARGS__)
#define LOG_TEXT(mode, format) printf("[%i:%02d " mode "] " LOGGER ": " format "\n", (int)floor(TIME_SECONDS / 60), TIME_SECONDS % 60)

#define DEBUG(format, ...) LOG("DEBUG", format, __VA_ARGS__)
#define INFO(format, ...) LOG("\033[34mINFO\033[0m", format, __VA_ARGS__)
#define WARN(format, ...) LOG("\033[33mWARN\033[0m", format, __VA_ARGS__)
#define ERROR(format, ...) LOG("\033[31mERROR\033[0m", format, __VA_ARGS__)

#define DEBUG_TEXT(format) LOG_TEXT("DEBUG", format)
#define INFO_TEXT(format) LOG_TEXT("\033[34mINFO\033[0m", format)
#define WARN_TEXT(format) LOG_TEXT("\033[33mWARN\033[0m", format)
#define ERROR_TEXT(format) LOG_TEXT("\033[31mERROR\033[0m", format)