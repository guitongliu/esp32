#include "CmdTCP.h"


void WIFI_Init()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_SSID, wifi_Password);

  // 配置静态 IP 地址
  IPAddress localIP(192, 168, 4, 2);   // 设备 IP !!!!!!!!!!!!要修改！！！！！！从4.2开始
  IPAddress gateway(192, 168, 4, 1);   // 网关（通常与服务器一致）
  IPAddress subnet(255, 255, 255, 0);   // 子网掩码
  WiFi.config(localIP, gateway, subnet);
  
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  
  // Serial.println("\nConnected to WiFi");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());
  
  
}


void ComdSet(uint8_t cmd_buffer[8])  //上位机指令识别共8个字节
{
  if(cmd_buffer[0]==head1&&cmd_buffer[1]==head2&&cmd_buffer[6]==End1&&cmd_buffer[7]==End2) 
  {
 
    switch (cmd_buffer[2]) //cmd_buffer[2]是指令位CMD
    {
    case cmd_Start: //CMD是cmd_start
    if(cmd_buffer[4]==0x11) //上位机发送指令开始读数据cmd_start，cmd_buffer[4]是CMD_Value
    {
        Serial.println("cmd_Start");
        attachInterrupt(19,getVal,ONHIGH); //ADS131 DRDY 19号脚置高的时候，调用getVal()函数(在main里),ADC读值
    }
      else if (cmd_buffer[4]==0x55)//上位机发送指令停止读数据cmd_Stop
      {
        Serial.println("cmd_Stop");
        detachInterrupt(19); //关读数据中断
        timerAlarmDisable(timer);      // 关闭ACC定时器
        readAccFlag = false;           // 清除加速度采集标志
        
        // 可选：停止数据发送
        timerAlarmDisable(timer_tran); // 关闭发送定时器
        SendFlag = false;              // 清除发送标志
      }        
        break;
    case cmd_Set_OSR: //CMD是cmd_Set_OSR, 设置采样率
        
        detachInterrupt(19); //关读数据中断
        Serial.println("cmd_Set_OSR");
        adc.setOsr(cmd_buffer[4]); //adc设置采样率尾cmd_buffer[4]的值
        Serial.println(adc.readReg(REG_CLOCK),HEX); //读adc寄存器，看OSR是否正确配置
        attachInterrupt(19,getVal,ONHIGH);//继续恢复读数据中断
        break;
    case cmd_Set_Filter: //设置DC_Block滤波器截止频率
        detachInterrupt(19);
        Serial.println("cmd_Set_Filter");   
        adc.writeRegister(THRSHLD_LSB, cmd_buffer[4]);
        Serial.println(adc.readReg(THRSHLD_LSB),HEX);
        attachInterrupt(19,getVal,ONHIGH);

        break;
    case cmd_Set_PGA: //设置每通道PGA增益
        detachInterrupt(19);
        Serial.println("cmd_Set_PGA"); 
        adc.setGain(cmd_buffer[4]);
        Serial.println(adc.readReg(REG_GAIN1),HEX);//REG_GAIN1:CH0-3
        Serial.println(adc.readReg(REG_GAIN2),HEX);//REG_GAIN2:CH4-7
        attachInterrupt(19,getVal,ONHIGH);
        break;
    default:
    Serial.println("n2n");
        break;
    }
  
  }
  else
  {
    Serial.println("non");
  }
}