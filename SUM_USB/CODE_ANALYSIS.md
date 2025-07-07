# ESP32 EMG系统代码详解

## 头文件和宏定义

```cpp
#include <WiFi.h>        // Arduino WiFi库
#include <esp_wifi.h>    // ESP-IDF WiFi底层接口
#include <bitset>        // C++ STL位集合，用于标记节点状态
#include <esp_timer.h>   // ESP-IDF高精度定时器

// 系统配置参数
#define NODE_MAXCOUNT 4                    // 支持的最大节点数量
#define DATA_PORT 12345                    // TCP数据传输端口
#define BUFFER_SLOTS 10                    // 环形缓冲区槽位数量
#define NODE_EXPECTED 4                    // 期望接收的节点数量
#define PACKET_SIZE 51                     // 网络数据包大小(2+1+48)
#define PACKET_PAYLOAD_SIZE 48             // NodeData实际数据大小
#define MAX_BUFFER_SIZE 2048               // 网络接收缓冲区大小
#define CMD_FRAME_SIZE 8                   // 命令帧固定大小
#define MAX_PACKETS_PER_NODE_PER_ITERATION 2  // 限制每轮处理包数，防止阻塞
```

## 数据结构设计

### NodeData结构体
```cpp
#pragma pack(push, 1)  // 强制1字节对齐，避免编译器自动填充
typedef struct {
  struct { uint8_t ch1[3]; } adc[8];  // 8通道24位ADC数据
  struct { float x, y, z; } imu[2];   // 2个IMU的xyz轴数据
} NodeData;  // 总大小: 8*3 + 2*3*4 = 48字节
#pragma pack(pop)
```

**设计说明**:
- 使用`#pragma pack(1)`确保结构体紧密排列
- ADC数据用3字节存储24位精度值
- IMU数据使用float类型存储浮点坐标

### TimeSlot结构体
```cpp
typedef struct {
  NodeData nodes[NODE_MAXCOUNT];         // 存储4个节点的完整数据
  std::bitset<NODE_MAXCOUNT> readyFlags; // 4位标志，标记节点数据就绪状态
} TimeSlot;
```

**设计说明**:
- `readyFlags`使用STL bitset，提供高效的位操作
- 每个时间槽可以存储一个完整的数据帧(所有节点的数据)

## 全局变量和缓冲区

### 双级缓冲机制
```cpp
TimeSlot timeSlots[BUFFER_SLOTS];           // 环形缓冲区，存储完整帧
volatile uint8_t currentWriteSlot = 0;      // 写指针，processData使用
volatile uint8_t currentReadSlot = 0;       // 读指针，sendCombinedData使用
TimeSlot activeCollectionSlotData;          // 活动收集槽，收集各节点数据
```

**设计说明**:
- `volatile`关键字确保多任务环境下的内存可见性
- 环形缓冲区实现生产者-消费者模式
- 双指针管理避免读写冲突

### 网络相关变量
```cpp
const IPAddress nodeIPs[NODE_MAXCOUNT] = {  // 预定义节点IP地址
  IPAddress(192,168,4,2), IPAddress(192,168,4,3),
  IPAddress(192,168,4,4), IPAddress(192,168,4,5)
};
WiFiClient clients[NODE_MAXCOUNT];          // 各节点的TCP客户端对象
WiFiServer server(DATA_PORT);               // TCP服务器

uint8_t netBuffer[NODE_MAXCOUNT][MAX_BUFFER_SIZE];  // 各节点网络接收缓冲区
size_t netBufLen[NODE_MAXCOUNT] = {0};             // 各缓冲区当前数据长度
```

## 核心算法实现

### 1. CRC16校验算法
```cpp
uint16_t crc16(const uint8_t* data, size_t len, uint16_t initial = 0xFFFF) {
  uint16_t crc = initial;
  for(size_t i=0; i<len; i++){
    crc ^= (uint16_t)data[i] << 8;    // 数据字节左移8位与CRC异或
    for(uint8_t j=0; j<8; j++){
      // 使用CCITT-16多项式 0x1021
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}
```

**算法说明**:
- 使用标准CCITT-16算法
- 多项式0x1021保证良好的错误检测能力
- 支持链式计算，可分段验证数据

### 2. 环形缓冲区操作
```cpp
bool isTimeSlotBufferFull() {
    return ((currentWriteSlot + 1) % BUFFER_SLOTS) == currentReadSlot;
}

bool isTimeSlotBufferEmpty() {
    return currentWriteSlot == currentReadSlot;
}
```

**算法说明**:
- 使用模运算实现环形索引
- 满缓冲区判断：写指针+1等于读指针
- 空缓冲区判断：写指针等于读指针

### 3. 数据处理流程 (processData)
```cpp
void processData(uint8_t nodeID, uint8_t* data) {
  if (nodeID >= NODE_MAXCOUNT) return;  // 边界检查

  // 1. 更新时间戳
  last_successful_process_time = micros();

  // 2. 复制数据到活动收集槽
  memcpy(&activeCollectionSlotData.nodes[nodeID], data, sizeof(NodeData));
  activeCollectionSlotData.readyFlags.set(nodeID);  // 设置节点就绪标志

  // 3. 检查是否收集完成
  if (activeCollectionSlotData.readyFlags.count() == NODE_EXPECTED) {
    // 4. 处理缓冲区满的情况
    uint8_t nextWriteSlotCandidate = (currentWriteSlot + 1) % BUFFER_SLOTS;
    if (nextWriteSlotCandidate == currentReadSlot) {
      // 缓冲区满，丢弃最老数据
      timeSlots[currentReadSlot].readyFlags.reset();
      currentReadSlot = (currentReadSlot + 1) % BUFFER_SLOTS;
    }

    // 5. 提交完整帧到环形缓冲区
    timeSlots[currentWriteSlot] = activeCollectionSlotData;
    currentWriteSlot = nextWriteSlotCandidate;
    
    // 6. 重置活动收集槽
    activeCollectionSlotData.readyFlags.reset();
    frames_committed++;
  }
}
```

**流程说明**:
1. **边界检查**: 防止非法节点ID
2. **数据复制**: 使用memcpy高效复制数据
3. **完成检查**: 使用bitset.count()检查是否所有节点就绪
4. **溢出处理**: LRU策略，丢弃最老数据
5. **原子提交**: 结构体赋值是原子操作
6. **状态重置**: 准备下一轮数据收集

### 4. 数据发送流程 (sendCombinedData)
```cpp
void sendCombinedData() {
  send_attempts++;
  if (isTimeSlotBufferEmpty()) return;  // 无数据直接返回

  TimeSlot& slotToSend = timeSlots[currentReadSlot];
  
  // 发送条件检查
  if(slotToSend.readyFlags.any()) {  // 有任何节点数据就发送
    size_t txIndex = 0;
    
    // 1. 构建包头
    uint8_t header[4] = {
      0xAA, 0x55,                               // 同步字
      (uint8_t)slotToSend.readyFlags.count(),   // 实际节点数
      (uint8_t)sizeof(NodeData)                 // 节点数据大小
    };
    memcpy(serialTxAggregatedBuffer + txIndex, header, sizeof(header));
    txIndex += sizeof(header);

    // 2. 计算包头CRC
    uint16_t crc = crc16(header, sizeof(header));

    // 3. 添加节点数据
    for(int i=0; i<NODE_MAXCOUNT; i++) {
      if(slotToSend.readyFlags.test(i)) {  // 只发送就绪节点的数据
        uint8_t id = i;
        serialTxAggregatedBuffer[txIndex++] = id;  // 节点ID
        memcpy(serialTxAggregatedBuffer + txIndex, 
               (uint8_t*)&slotToSend.nodes[i], sizeof(NodeData));
        txIndex += sizeof(NodeData);

        // 4. 更新CRC
        crc = crc16(&id, 1, crc);
        crc = crc16((uint8_t*)&slotToSend.nodes[i], sizeof(NodeData), crc);
      }
    }

    // 5. 添加CRC校验
    uint8_t crcBytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    memcpy(serialTxAggregatedBuffer + txIndex, crcBytes, sizeof(crcBytes));
    txIndex += sizeof(crcBytes);
    
    // 6. 单次发送
    Serial.write(serialTxAggregatedBuffer, txIndex);
    send_successes++;
  }

  // 7. 清理并前进到下一槽
  slotToSend.readyFlags.reset();
  currentReadSlot = (currentReadSlot + 1) % BUFFER_SLOTS;
}
```

**发送包格式**:
```
[0xAA][0x55][节点数][数据大小] + [节点ID][NodeData] × N + [CRC_H][CRC_L]
```

## 网络任务实现 (taskNetwork)

### select()多路复用机制
```cpp
void taskNetwork(void *pvParam) {
  fd_set read_fds;      // 文件描述符集合
  int max_fd = 0;       // 最大文件描述符
  struct timeval tv;    // 超时设置

  while(1) {
    // 1. 处理新连接
    WiFiClient newClient = server.available();
    if(newClient){
      IPAddress ip = newClient.remoteIP();
      for(int i=0; i<NODE_MAXCOUNT; i++){
        if(ip == nodeIPs[i]){  // 基于IP地址识别节点
          if(clients[i].connected()) clients[i].stop();  // 关闭旧连接
          clients[i] = newClient;
          break;
        }
      }
    }

    // 2. 准备select()参数
    FD_ZERO(&read_fds);   // 清空描述符集
    max_fd = 0;
    for(int i=0; i<NODE_MAXCOUNT; i++){
      if(clients[i].connected()){
        int sock_fd = clients[i].fd();
        if(sock_fd != -1) {
          FD_SET(sock_fd, &read_fds);  // 添加到监听集合
          if(sock_fd > max_fd) max_fd = sock_fd;
        }
      }
    }

    // 3. 设置超时
    tv.tv_sec = 0;
    tv.tv_usec = 10;  // 10微秒超时，保证高响应性

    // 4. 多路复用等待
    if (max_fd > 0) {
      int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
      
      if (ret > 0) {  // 有数据可读
        for(int i=0; i<NODE_MAXCOUNT; i++){
          if(clients[i].connected()){
            int sock_fd = clients[i].fd();
            if(sock_fd != -1 && FD_ISSET(sock_fd, &read_fds)){
              // 5. 读取数据
              size_t avail = clients[i].available();
              if(avail > 0){
                size_t readLen = clients[i].read(
                  netBuffer[i] + netBufLen[i],
                  min(avail, MAX_BUFFER_SIZE - netBufLen[i])
                );
                netBufLen[i] += readLen;

                // 6. 解析数据包
                int packets_processed = 0;
                while(netBufLen[i] >= 3 && 
                      packets_processed < MAX_PACKETS_PER_NODE_PER_ITERATION) {
                  
                  // 解析包长度
                  uint16_t pktLen = (netBuffer[i][0] << 8) | netBuffer[i][1];
                  size_t totalPktSize = 2 + 1 + pktLen;  // 长度+ID+数据

                  if(netBufLen[i] >= totalPktSize) {
                    if(pktLen == sizeof(NodeData)) {
                      uint8_t nodeID = netBuffer[i][2];
                      processData(nodeID, &netBuffer[i][3]);
                      packets_processed++;
                    }
                    
                    // 移除已处理的数据
                    size_t remain = netBufLen[i] - totalPktSize;
                    memmove(netBuffer[i], netBuffer[i]+totalPktSize, remain);
                    netBufLen[i] = remain;
                  } else {
                    break;  // 数据不完整，等待更多数据
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}
```

**性能优化要点**:
1. **select()多路复用**: 避免轮询，提高CPU效率
2. **10微秒超时**: 平衡响应性和CPU使用率
3. **包数量限制**: 防止单个节点阻塞其他节点
4. **memmove()优化**: 高效的内存块移动

## 定时器回调实现

```cpp
void sendTimerCallback(void* arg) {
    sendCombinedData();  // 每1ms调用一次发送函数
}
```

**实时性保证**:
- 使用ESP-IDF高精度定时器
- 1KHz固定频率，误差小于1微秒
- 独立任务执行，不受其他任务影响

## 系统初始化 (setup)

```cpp
void setup() {
  // 1. 初始化USB CDC
  Serial.begin(115200);

  // 2. 初始化数据结构
  activeCollectionSlotData.readyFlags.reset();
  for (int i = 0; i < BUFFER_SLOTS; ++i) {
    timeSlots[i].readyFlags.reset();
  }

  // 3. 创建任务
  xTaskCreatePinnedToCore(taskNetwork, "net", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(taskUSBCommand, "usb_cmd", 4096, NULL, 4, NULL, 0);

  // 4. 配置定时器
  const esp_timer_create_args_t timer_args = {
    &sendTimerCallback, nullptr, ESP_TIMER_TASK, "send_timer"
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &send_timer_handle));
  ESP_ERROR_CHECK(esp_timer_start_periodic(send_timer_handle, 1000));

  // 5. 启动WiFi热点
  WiFi.softAP("ESP32_SUM", "123456789", 6, 0, NODE_MAXCOUNT);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  server.begin();
  esp_wifi_set_ps(WIFI_PS_NONE);  // 关闭省电模式
}
```

**初始化顺序**:
1. **串口初始化**: 建立调试输出通道
2. **数据结构清零**: 确保初始状态正确
3. **任务创建**: 分配到不同CPU核心
4. **定时器配置**: 设置1KHz发送频率
5. **网络启动**: 建立WiFi热点和TCP服务器

---

*本文档提供了ESP32 EMG系统的详细代码解析，涵盖了从数据结构设计到算法实现的所有关键技术点。*
