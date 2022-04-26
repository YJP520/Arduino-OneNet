 /*
  Adeept DHT11 Module library V1.0
  2017 Copyright (c) Adeept Technology Inc.  All right reserved.
  Author: TOM
*/

/*
 * 
 * TIME       :     2021.04.06
 * Author     :     YU.J.P
 * Project    :     ARDUINO + ESP8266 --> ONENET
 * 
 */

#ifndef DHT11_H
#define DHT11_H

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#define DHT11LIB_VERSION "V1.0"

#define DHTLIB_OK        0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT  -2

class DHT11
{
public:
  int read(int pin);
  int humidity;
  int temperature;
};
#endif
