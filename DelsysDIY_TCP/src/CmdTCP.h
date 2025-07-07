#ifndef CmdTCP_H
#define CmdTCP_H

#include "main.h"
#include "ADS131M02.h"
// #include <WiFi.h>  //wifi功能需要的库
// #include <Arduino.h>

extern const char* wifi_SSID ;  //存储AP的名称信息
extern const char* wifi_Password ;  //存储AP的密码信息
extern const uint16_t Data_port ;  //数据端口号
extern const uint16_t Cmd_Rx_port ;  //指令端口号
extern const uint16_t Cmd_Tx_port ;  //指令端口号

extern WiFiClient tcpClient; 
extern IPAddress remoteIP;
extern ADS131M02 adc;
extern bool isConnected;

extern hw_timer_t* timer;          // 声明外部变量
extern hw_timer_t* timer_tran;
extern bool readAccFlag;
extern bool SendFlag;

#define head1 0x56 //上位机命令帧头1
#define head2 0x55  //上位机命令帧头2
#define End1 0xe6 //上位机命令帧尾1
#define End2 0xe5 //上位机命令帧尾2

#define cmd_Start 0x11   
#define cmd_Set_OSR 0x22
#define cmd_Set_Filter 0x33
#define cmd_Set_PGA 0x44


void WIFI_Init();
void TCPTx(char* buf);
void ComdSet(uint8_t cmd_buffer[8]);
#endif