#pragma once

#ifdef _WIN32
#include <ws2tcpip.h>
#else 
#include <arpa/inet.h>
#endif

inline int16_t WJ_SWAP_INT16(int16_t value)
{
  uint8_t *v = (uint8_t *)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 
#endif

#include <math.h>

#define DEGREE_TO_RADIAN(deg) ((deg)*M_PI / 180)
#define RADIAN_TO_DEGREE(deg) ((deg)*180 / M_PI)
