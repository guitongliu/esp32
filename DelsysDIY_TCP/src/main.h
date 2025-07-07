#ifndef MAIN_H
#define MAIN_H

#include <WiFi.h>  //wifi功能需要的库
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include "CmdTCP.h"
#include "ADS131M02.h"
#include "esp_sleep.h" // 确保包含 ESP Sleep 库



#define head1 0x56
#define head2 0x55
#define End1 0xe6
#define End2 0xe5

#define cmd_Start 0x11
#define cmd_Set_OSR 0x22
#define cmd_Set_Filter 0x33
#define cmd_Set_PGA 0x44

void getVal();
void setup();


#endif