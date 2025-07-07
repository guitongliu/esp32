#include "main.h"
#define DRDY 19
#define CS 5
#define SCLK 14
#define DIN 12
#define DOUT 13
#define SDA_PIN 26
#define SCL_PIN 25

#pragma pack(push, 1)
typedef struct {
  // uint32_t timestamp;
  struct { uint8_t ch1[3]; } adc[8];
  struct { float x, y, z; } imu[2];
} NodeData; // 48 bytes
#pragma pack(pop)
NodeData nodeData; 
#define IMU_BUFFER_SIZE 2 // IMU数据缓冲区大小，在修改数据包结构时需要同步修改
uint8_t imuIndex = 0;

// #define BUFFER_SIZE 100 // 假设缓冲区大小为100
TaskHandle_t TASK_HandleOne = NULL;

WiFiClient tcpClient;   // TCP客户端对象
IPAddress serverIP(192, 168, 4, 1);
//(192, 168, 4, 2)
const char* wifi_SSID = "ESP32_SUM";  // 连接到汇总板的SSID
const char* wifi_Password = "123456789";  // 汇总板的密码
const uint16_t Data_port=12345;  //数据端口号
const uint16_t Cmd_Rx_port=667;  //指令端口号
const uint16_t Cmd_Tx_port=666;  //指令端口号

const uint8_t nodeID = 0;  // 节点ID从0开始!!!!!!!!!!!!!!!!!!!!!!!!!!!!!和IP同步要修改


ADS131M02 adc(8192000, SCLK, DOUT, DIN, CS, DRDY); //实例化ADS131

uint16_t flag=0;
uint16_t flag_acc=0;
const int CHANNEL_SIZE = 8; //EMG采样点数量，修改数据包结构时也需要同步修改
const int PACKET_NUM = 1;


char tx_buffer[10];
char rx_buffer[20];
uint8_t cmdBuffer[8];      // 指令接收缓冲区
size_t cmdIndex = 0;       // 当前接收位置

int Data_length;
float x, y, z;

hw_timer_t * timer = NULL;  // 定时器指针
hw_timer_t * timer_tran = NULL;

bool readAccFlag = false;  // 用于标识是否需要读取加速度数据
bool SendFlag = false; 

// 新增连接状态标志
bool isConnected = false;

// 添加低功耗相关变量
const unsigned long WIFI_CONNECT_TIMEOUT = 5000;  // WiFi连接超时时间（5秒）
const unsigned long WIFI_RETRY_INTERVAL = 1000; // WiFi重试间隔（1秒）
unsigned long connectionStartTime = 0;   // 开始尝试连接的时间点
unsigned long lastRetryAttempt = 0;      // 上次尝试连接的时间点
bool isInLowPowerMode = false; // 标记是否已进入低功耗模式（主要用于避免重复调用）


// 在全局作用域
volatile uint32_t tran_call_count = 0; // volatile确保在不同上下文（主循环和可能的其他任务）中正确访问
unsigned long last_tran_print_time = 0;
// 进入低功耗模式的函数
void enterLowPowerMode() {
  if (!isInLowPowerMode) {
    isInLowPowerMode = true; // 先设置标志，防止重入
    Serial.println("WiFi connection timeout or not connected. Entering low power mode...");
    Serial.flush(); // 确保串口消息发送完毕

    // 停止 TCP 客户端 (如果活动)
    if (tcpClient.connected()) {
        tcpClient.stop();
    }
    isConnected = false; // 标记为未连接

    // 关闭WiFi
    WiFi.disconnect(true); // true: 强制断开并清除配置
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi turned off.");

    // 停止定时器 (如果已启动) - 确保在睡眠前停止硬件定时器
    if (timer) {
      timerAlarmDisable(timer);
      timerEnd(timer); // 释放定时器资源
      timer = NULL;    // 标记为未初始化
      Serial.println("Timer 0 disabled and ended.");
    }
    if (timer_tran) {
      timerAlarmDisable(timer_tran);
      timerEnd(timer_tran); // 释放定时器资源
      timer_tran = NULL;   // 标记为未初始化
      Serial.println("Timer 1 disabled and ended.");
    }

    // --- 配置定时器唤醒 ---
    // 参数是唤醒时间，单位是微秒 (microseconds)
    // 20 秒 = 20 * 1,000,000 微秒
    uint64_t sleepTimeUs = 5 * 1000000;
    Serial.printf("Enabling timer wakeup for %llu us (%d seconds).\n", sleepTimeUs, (int)(sleepTimeUs / 1000000));
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    // ----------------------

    Serial.println("Entering deep sleep now.");
    Serial.flush(); // 再次确保日志输出
    esp_deep_sleep_start(); // 进入深度睡眠，执行到这里会停止，唤醒后会重启
  }
}

void IRAM_ATTR onTimer() {
  // 设置标志位，表示需要在主循环中读取加速度数据
  readAccFlag = true;
}

void IRAM_ATTR onTRANTimer() {
  // 设置标志位，表示需要在主循环中读取加速度数据
  SendFlag = true;
}

void read_acc() 
{
    // 存储加速度数据到缓冲区
    IMU.readAcceleration(x, y, z);
    nodeData.imu[imuIndex].x = x;
    nodeData.imu[imuIndex].y = y;
    nodeData.imu[imuIndex].z = z;
    imuIndex = (imuIndex + 1) % IMU_BUFFER_SIZE;
    readAccFlag = false;  // 清除标志位
}

void getVal()
{
  if (flag < CHANNEL_SIZE * PACKET_NUM) {
    // 读取新的肌电数据
    adc.read_ADC(nodeData.adc[flag].ch1);  // 读取3字节的数据（一个采样点）
    flag++;  // 增加标志位
  }
}

// 在 Tran() 函数中
void Tran() {
  if (!tcpClient.connected()) return;

  uint32_t tran_start_time = micros();

  uint16_t packetLen = sizeof(NodeData); // 48 bytes
  uint8_t packet_id = nodeID;

  // 创建一个发送缓冲区
  size_t total_packet_size = 2 + 1 + packetLen; // header (2) + id (1) + payload (48) = 51
  uint8_t sendBuffer[total_packet_size]; // 或者用动态分配，但对于固定小包，栈上分配更快

  // 组装包头 (大端)
  sendBuffer[0] = (uint8_t)(packetLen >> 8);
  sendBuffer[1] = (uint8_t)(packetLen & 0xFF);
  
  // 组装ID
  sendBuffer[2] = packet_id;

  // 组装数据负载
  memcpy(&sendBuffer[3], (uint8_t*)&nodeData, packetLen);

  // 调用一次 write 发送整个包
  size_t written_bytes = tcpClient.write(sendBuffer, total_packet_size);
  
  // tcpClient.flush(); // 可选：尝试flush，但通常TCP自己会处理。有时可能有用，有时可能增加延迟。先不加。

  // uint32_t tran_duration = micros() - tran_start_time;
  // if (tran_duration > 500) { // 阈值可以根据情况调整
  //     Serial.printf("Node %d - Tran took: %lu us, Sent: %d/%u\n", 
  //                   nodeID, tran_duration, written_bytes, total_packet_size);
  // }
  // if (written_bytes != total_packet_size) {
  //     Serial.printf("Node %d - TCP Write Error! Sent: %d/%u\n",
  //                   nodeID, written_bytes, total_packet_size);
  // }

  tran_call_count++;
}


// 修改后的指令处理任务
void TASK_ONE(void *param) {
  const TickType_t xDelay = 20 / portTICK_PERIOD_MS;
  
  while (1) {
    if (!isConnected) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    // 优化后的指令接收逻辑
    while (tcpClient.connected() && tcpClient.available()) {
      // 计算剩余需要接收的字节数
      size_t remaining = sizeof(cmdBuffer) - cmdIndex;
      
      // 安全读取（防止溢出）
      int readLen = tcpClient.readBytes(
        cmdBuffer + cmdIndex, 
        min(remaining, (size_t)tcpClient.available())
      );
      
      if(readLen > 0) {
        cmdIndex += readLen;
        
        // 完整接收后处理
        if(cmdIndex == sizeof(cmdBuffer)) {
          ComdSet(cmdBuffer);  // 处理指令
          cmdIndex = 0;        // 重置缓冲区
        }
      }
      
      // 防止死循环
      if(readLen <= 0) break;
    }
    
    vTaskDelay(xDelay);
  }
}


void setup() {
  // put your setup code here, to run once:
  gpio_install_isr_service(0); // 参数 0 表示默认情况下没有额外的标志
  Serial.begin(115200);  //开启串口，波特率为115200 
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!IMU.begin()) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("NO LSM9DS1!");
    while (1);
  }
 /*建立子线程，调用TASK_ONE()函数,接收上位机指令帧*/
  xTaskCreate( 
                    TASK_ONE,          /* 任务函数 */
                    "TaskOne",         /* 任务名 */
                    8*1024,            /* 任务栈大小，根据需要自行设置*/
                    NULL,              /* 参数，入参为空 */
                    1,                 /* 优先级 */
                    &TASK_HandleOne);  /* 任务句柄 */
      
  Serial.println("Setting SPI");
  adc.spi = new SPIClass(VSPI); //ESP32 SPI配置
  Serial.println("Booting SPI");
  adc.init();
  Serial.println("setOsr");
  adc.setOsr(OSR_1024);
  adc.setGain(CHANNEL_PGA_4);
  adc.writeRegister(THRSHLD_LSB, 0x04);
  attachInterrupt(19,getVal,ONHIGH); //读EMG输出中断

  Serial.println(adc.SpiClk);

  WIFI_Init();


  // 初始化连接计时器
  connectionStartTime = millis();
  lastRetryAttempt = connectionStartTime; // 立即开始第一次尝试

  timer = timerBegin(0, 80, true); // 定时器 0，分频 80
  timerAttachInterrupt(timer, &onTimer, true);  // 连接中断
  timerAlarmWrite(timer, 1200, true);  // 每 1.2ms 执行一次1.2ms -> ~833Hz
  timerAlarmEnable(timer);  // 启动定时器

  timer_tran = timerBegin(1, 80, true); // 定时器 0，分频 80
  timerAttachInterrupt(timer_tran, &onTRANTimer, true);  // 连接中断
  timerAlarmWrite(timer_tran, 2000, true);  // 每 0.5ms，2ms 执行一次
  timerAlarmEnable(timer_tran);  // 启动定时器
}

void loop() 
{
  unsigned long currentTime = millis();
  
  // 连接管理
// --- 连接管理 ---
if (!isConnected) {
  // 1. 检查总连接超时
  if (currentTime - connectionStartTime >= WIFI_CONNECT_TIMEOUT) {
    enterLowPowerMode(); // 超时，进入低功耗模式 (此函数会停止执行并重启)
    // 注意：一旦调用 enterLowPowerMode 并进入 deep sleep，下面的代码不会执行
  }

  // 2. 检查是否到了重试时间
  if (currentTime - lastRetryAttempt >= WIFI_RETRY_INTERVAL) {
    lastRetryAttempt = currentTime; // 更新上次尝试时间

    // 检查 WiFi 物理层连接状态 (可选，但推荐)
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected. IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Attempting to connect to TCP Server: ");
      Serial.print(serverIP);
      Serial.print(":");
      Serial.println(Data_port);

      // 尝试连接 TCP 服务器
      if (tcpClient.connect(serverIP, Data_port)) {
        isConnected = true;
        // tcpClient.setNoDelay(true);
        Serial.println("TCP connection successful!");
        // 连接成功，重置命令缓冲区索引
        cmdIndex = 0;

      } else {
        Serial.println("TCP connection failed.");
        // TCP 连接失败，WiFi 物理层可能还是连接的，下次继续尝试 TCP 连接
      }
    } else {
       // WiFi 物理层未连接，打印状态并等待下次重试
       Serial.print("WiFi not connected. Status: ");
       Serial.print(WiFi.status());
       Serial.println(". Retrying...");
       // 可以选择在这里再次调用 WiFi.begin()，但通常 WiFi 库会在后台自动重试
       // WiFi.begin(wifi_SSID, wifi_Password);
    }
  }
  // 如果未连接，不执行后面的数据处理和发送逻辑
  // 加一个小延时避免 CPU 空转太快 (如果其他任务不运行时)
  // vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 延时
  delay(10); // 简单的延时

  return; // 跳过循环的其余部分
}

  if(readAccFlag) read_acc();
  if(flag>=CHANNEL_SIZE*PACKET_NUM) flag=0;

  if (SendFlag) {
    Tran();
    SendFlag = false;
  }

  // 连接断开时重置状态
  if (!tcpClient.connected()) {
    tcpClient.stop();
    isConnected = false;
    cmdIndex = 0;
    connectionStartTime = currentTime;
    lastRetryAttempt = currentTime;

  }
    // 每秒打印一次Tran()调用频率
  // if (currentTime - last_tran_print_time >= 1000) {
  //   Serial.printf("Node %d - Tran calls per second: %u\n", nodeID, tran_call_count);
  //   tran_call_count = 0; // 重置计数器
  //   last_tran_print_time = currentTime;
  // }
}

