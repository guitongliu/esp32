#include <WiFi.h>
#include <esp_wifi.h>
#include <bitset>
#include <esp_timer.h> // ESP-IDF高性能定时器，更精确
// #include <driver/uart.h> // 不再直接使用UART驱动，可以移除

#define NODE_MAXCOUNT 4
// #define SERIAL_BAUDRATE 460800  // USB CDC 不使用传统的波特率概念，此定义可以保留用于 Serial.begin() 参数，但实际速度由USB决定
#define DATA_PORT 12345
#define BUFFER_SLOTS 10          // 适当增加缓冲槽数量
#define NODE_EXPECTED 4
// #define PACKET_SIZE 147         // 2+1+144字节
#define PACKET_SIZE 51         // 2+1+48字节
#define PACKET_PAYLOAD_SIZE 48   // NodeData的实际大小
#define MAX_NET_BUFFER_SIZE 2048 // 每个节点的网络接收缓冲区
#define MAX_BUFFER_SIZE 2048
#define CMD_FRAME_SIZE 8
#define MAX_PACKETS_PER_NODE_PER_ITERATION 2 // 比如，每次只处理2个数据包
uint8_t cmdBuffer[CMD_FRAME_SIZE];
size_t cmdIndex = 0;

#pragma pack(push, 1)
typedef struct {
  struct { uint8_t ch1[3]; } adc[8];
  struct { float x, y, z; } imu[2];
} NodeData;

typedef struct {
  NodeData nodes[NODE_MAXCOUNT];  // 存储来自多个节点的数据
  std::bitset<NODE_MAXCOUNT> readyFlags;
} TimeSlot;
#pragma pack(pop)

TimeSlot timeSlots[BUFFER_SLOTS];
// volatile uint8_t currentSlot = 0;
// volatile uint8_t sendSlot = 0;

volatile uint8_t currentWriteSlot = 0; // `processData` 写入的槽位
volatile uint8_t currentReadSlot = 0;  // `sendCombinedData` 读取的槽位

TimeSlot activeCollectionSlotData;

// 网络配置和客户端管理
const char* ssid = "ESP32_SUM";
const char* password = "123456789";
const IPAddress nodeIPs[NODE_MAXCOUNT] = {
  IPAddress(192,168,4,2),
  IPAddress(192,168,4,3),
  IPAddress(192,168,4,4),
  IPAddress(192,168,4,5)
};
WiFiClient clients[NODE_MAXCOUNT];
WiFiServer server(DATA_PORT);

// 网络任务专用缓冲区
uint8_t netBuffer[NODE_MAXCOUNT][MAX_BUFFER_SIZE];
size_t netBufLen[NODE_MAXCOUNT] = {0};


// ESP-IDF 定时器句柄
esp_timer_handle_t send_timer_handle;

// 调试计数器和计时器
static uint32_t timer_cb_calls = 0;
static uint32_t last_timer_print_micros = 0;

static uint32_t send_attempts = 0;
static uint32_t send_successes = 0;
static uint32_t last_send_print_micros = 0;
static uint32_t frames_committed = 0;
static uint32_t last_commit_print_micros = 0;
static uint32_t data_processed_count[NODE_MAXCOUNT] = {0}; // 新增：统计每个节点的数据包处理次数


// 全局变量，或作为 sendTimerCallback 的静态变量
static uint64_t last_commit_time = 0;
// static std::bitset<NODE_MAXCOUNT> last_committed_flags; // 记录上次提交时的flags，避免重复提交相同的部分数据
const uint64_t FORCE_COMMIT_TIMEOUT_US = 5000; // 5ms，如果5ms内没有自然凑齐，就考虑强制处理

uint64_t last_successful_process_time = 0;

// Helper function to check if buffer is full
bool isTimeSlotBufferFull() {
    return ((currentWriteSlot + 1) % BUFFER_SLOTS) == currentReadSlot;
}

// Helper function to check if buffer is empty
bool isTimeSlotBufferEmpty() {
    return currentWriteSlot == currentReadSlot;
}

// 添加CRC计算 (与上位机保持一致)
uint16_t crc16(const uint8_t* data, size_t len, uint16_t initial = 0xFFFF) {
  uint16_t crc = initial;
  for(size_t i=0; i<len; i++){
    crc ^= (uint16_t)data[i] << 8;
    for(uint8_t j=0; j<8; j++){
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// 定义一个足够大的缓冲区用于单次Serial.write
#define MAX_SERIAL_TX_PACKET_SIZE (4 + NODE_MAXCOUNT * (1 + sizeof(NodeData)) + 2)
uint8_t serialTxAggregatedBuffer[MAX_SERIAL_TX_PACKET_SIZE];

void sendCombinedData() {
  send_attempts++;
  if (isTimeSlotBufferEmpty()) {
  // Serial.println("DEBUG: sendCombinedData called, but buffer is empty."); // 减少打印频率
    return;
  }
    // 测量并打印发送尝试和成功的频率
  // if (micros() - last_send_print_micros >= 1000000) { // 每1秒打印一次
  //   // Serial.printf("Send Stats: %lu attempts, %lu successes in 1s. Actual send freq: %.2f Hz\n", send_attempts, send_successes, (float)send_successes);
  //   send_attempts = 0;
  //   send_successes = 0;
  //   last_send_print_micros = micros();
  // }
  if (isTimeSlotBufferEmpty()) { // 如果读指针追上了写指针，说明没有新数据可发
    return;
  }

  TimeSlot& slotToSend = timeSlots[currentReadSlot];

  // // 打印当前槽位的信息，无论是否发送
  // Serial.printf("DEBUG: Considering slot %d. Flags: %s, Nodes Ready: %d/%d\n",
  //               currentReadSlot, slotToSend.readyFlags.to_string().c_str(),
  //               slotToSend.readyFlags.count(), NODE_EXPECTED);

  // 只有当前槽位准备好的节点数量达到预期才发送
  // 如果希望即使不完整也发送，可以修改或移除这个条件
  // if(slotToSend.readyFlags.count() >= NODE_EXPECTED) { // 或者修改为 slotToSend.readyFlags.any() 如果希望发送任何收到的数据
  if(slotToSend.readyFlags.any()) { // <--- 修改在这里！
    size_t txIndex = 0;
    // 包头: 0xAA, 0x55, 节点数量, 数据大小
    uint8_t header[4] = {0xAA, 0x55,
                        (uint8_t)slotToSend.readyFlags.count(), // 实际收到的节点数量
                        (uint8_t)sizeof(NodeData)}; // 每个节点数据的大小
    memcpy(serialTxAggregatedBuffer + txIndex, header, sizeof(header));
    txIndex += sizeof(header);

    uint16_t crc = crc16(header, sizeof(header));

    // 发送节点数据并更新CRC
    for(int i=0; i<NODE_MAXCOUNT; i++) {
      if(slotToSend.readyFlags.test(i)) {
        uint8_t id = i;
        serialTxAggregatedBuffer[txIndex++] = id;
        memcpy(serialTxAggregatedBuffer + txIndex, (uint8_t*)&slotToSend.nodes[i], sizeof(NodeData));
        txIndex += sizeof(NodeData);

        crc = crc16(&id, 1, crc);
        crc = crc16((uint8_t*)&slotToSend.nodes[i], sizeof(NodeData), crc);
      }
    }

    // 发送CRC
    uint8_t crcBytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    memcpy(serialTxAggregatedBuffer + txIndex, crcBytes, sizeof(crcBytes));
    txIndex += sizeof(crcBytes);
    Serial.write(serialTxAggregatedBuffer, txIndex);
    // Serial.printf("Sent slot %d with %d nodes.\n", currentReadSlot, slotToSend.readyFlags.count());
    

    // **** 测量 Serial.write 的耗时 ****
    // uint32_t start_write_micros = micros();
    // Serial.write(serialTxAggregatedBuffer, txIndex);
    // uint32_t end_write_micros = micros();

    // Serial.printf("DEBUG: Sent slot %d, %d nodes, %zu bytes. Serial.write took %lu us.\n",
    //               currentReadSlot, slotToSend.readyFlags.count(), txIndex,
    //               end_write_micros - start_write_micros);
    send_successes++;
  } else {
    // 即使不满足发送条件，也视为该槽位已处理，继续前进
    // Serial.printf("Slot %d (%s) not sent (only %d/%d nodes ready).\n", currentReadSlot, slotToSend.readyFlags.to_string().c_str(), slotToSend.readyFlags.count(), NODE_EXPECTED);
  }

  // 数据发送（或决定不发送）后，清空当前槽位并切换到下一个槽位
  slotToSend.readyFlags.reset(); // 清空标志，表示此槽位已处理
  currentReadSlot = (currentReadSlot + 1) % BUFFER_SLOTS; // 切换到下一个槽位
}


// sendTimerCallback 现在只负责触发 sendCombinedData
void sendTimerCallback(void* arg) {
    // uint32_t cb_start_micros = micros();

    // // 检查是否需要基于超时强制处理 activeCollectionSlotData
    // // (这个逻辑也可以放在 processData 中，但放在定时器里有周期性检查的意味)
    // bool collection_timeout = false;
    // if (activeCollectionSlotData.readyFlags.any() && // 如果当前正在收集数据
    //     (micros() - last_successful_process_time > FORCE_COMMIT_TIMEOUT_US)) { // last_successful_process_time 需要在 processData 中每次收到数据时更新
    //     // 或者更简单：如果 activeCollectionSlotData.readyFlags 很久没变过了，或者上次提交后很久了
    //     // 这里的超时逻辑需要仔细设计，避免与 processData 中的提交竞争
    //     // 一个更稳妥的超时：在 processData 中，如果一个新包来了，发现上一个包（任意节点）是很久以前的，
    //     // 并且 activeCollectionSlotData 还没有满，可以认为之前的收集“停滞”了。

    //     // 简化版超时：如果 activeCollectionSlotData 中有数据，并且距离上次成功提交已经很久了
    //     // if (activeCollectionSlotData.readyFlags.any() && (micros() - last_commit_time > FORCE_COMMIT_TIMEOUT_US)) {
    //     //      Serial.printf("TIMER: Collection timeout suspected. Flags: %s. Forcing commit/reset.\n", activeCollectionSlotData.readyFlags.to_string().c_str());
    //     //      // 决定是提交部分，还是直接reset
    //     //      // 提交部分（示例）：
    //     //      if (activeCollectionSlotData.readyFlags.count() >= 2) { // 至少有2个节点
    //     //         uint8_t nextWriteSlotCandidate = (currentWriteSlot + 1) % BUFFER_SLOTS;
    //     //         if (nextWriteSlotCandidate == currentReadSlot) { /* Buffer full handling */ }
    //     //         timeSlots[currentWriteSlot] = activeCollectionSlotData;
    //     //         // 注意：这里提交的是 activeCollectionSlotData，它的 readyFlags 可能不是全1
    //     //         // sendCombinedData 需要能处理这种情况，或者在这里修改 readyFlags 只保留真实有效的
    //     //         currentWriteSlot = nextWriteSlotCandidate;
    //     //         frames_committed++; // 算作一次提交
    //     //      }
    //     //      activeCollectionSlotData.readyFlags.reset();
    //     //      last_commit_time = micros(); // 更新提交时间，即使是部分提交或仅重置
    //     // }
    // }


    // --- 原有的发送逻辑 ---
    sendCombinedData(); // 尝试从 timeSlots 发送

    // uint32_t cb_end_micros = micros();
    // uint32_t cb_duration = cb_end_micros - cb_start_micros;
    // if (cb_duration > 950) { // 略小于1000，留点余量
    //     Serial.printf("WARN: sendTimerCallback took %lu us!\n", cb_duration);
    // }
}

void processData(uint8_t nodeID, uint8_t* data) {
  if (nodeID >= NODE_MAXCOUNT) return;

  last_successful_process_time = micros(); // 或者 esp_timer_get_time()，保持一致性

  // 写入当前的活动收集槽位
  memcpy(&activeCollectionSlotData.nodes[nodeID], data, sizeof(NodeData));
  activeCollectionSlotData.readyFlags.set(nodeID);
  data_processed_count[nodeID]++; // 统计该节点数据处理次数

  // ***** 详细打印标志位变化 *****
  // Serial.printf("PData: Node %d arrived. Flags: %s\n", nodeID, activeCollectionSlotData.readyFlags.to_string().c_str());

  // 检查是否所有节点的数据都已收集完毕
    if (activeCollectionSlotData.readyFlags.count() == NODE_EXPECTED) { // 或者 .all()
        // 所有节点数据都已到达，将 activeCollectionSlotData 提交到发送环形缓冲区
        uint8_t nextWriteSlotCandidate = (currentWriteSlot + 1) % BUFFER_SLOTS;

        if (nextWriteSlotCandidate == currentReadSlot) {
            // 缓冲区满！丢弃最老的数据帧来腾出空间
            // Serial.printf("CRITICAL: TimeSlot buffer full! WriteSlot: %d, ReadSlot: %d. Oldest data in slot %d dropped.\n", currentWriteSlot, currentReadSlot, currentReadSlot);
            timeSlots[currentReadSlot].readyFlags.reset(); // 清理被覆盖的槽位
            currentReadSlot = (currentReadSlot + 1) % BUFFER_SLOTS;
            // 注意：这里覆盖了最老的数据。如果这对你的应用不可接受，需要其他策略。
        }

        timeSlots[currentWriteSlot] = activeCollectionSlotData; // 结构体赋值
        // Serial.printf("DEBUG: Committed complete frame to slot %d. Flags: %s\n", currentWriteSlot, activeCollectionSlotData.readyFlags.to_string().c_str());
        currentWriteSlot = nextWriteSlotCandidate;

        // 清空 activeCollectionSlotData 的标志，准备接收下一组数据
        activeCollectionSlotData.readyFlags.reset();
        frames_committed++;
    }


    if (micros() - last_commit_print_micros >= 1000000) { // 每秒打印
      Serial.printf("Frames committed to buffer in 1s: %u. ", frames_committed);
      Serial.printf("ActiveCollectionFlags: %s. ", activeCollectionSlotData.readyFlags.to_string().c_str());
      Serial.print("Node data counts: ");
      for(int i=0; i<NODE_MAXCOUNT; i++) {
        Serial.printf("[%d]:%u ", i, data_processed_count[i]);
        data_processed_count[i] = 0; // 重置计数器
      }
      Serial.println();
      frames_committed = 0;
      last_commit_print_micros = micros();
    }
  // Serial.printf("Node %d data received. Active flags: %s\n", nodeID, activeCollectionSlotData.readyFlags.to_string().c_str());
}

// taskNetwork (保持之前我们优化过的版本)
void taskNetwork(void *pvParam) {
  fd_set read_fds;
  int max_fd = 0;
  struct timeval tv;

  while(1) {
    WiFiClient newClient = server.available();
    if(newClient){
      IPAddress ip = newClient.remoteIP();
      for(int i=0; i<NODE_MAXCOUNT; i++){
        if(ip == nodeIPs[i]){
          if(clients[i].connected()) clients[i].stop();
          clients[i] = newClient;
          // clients[i].setNoDelay(true);
          Serial.printf("Client %d connected from %s\n", i, ip.toString().c_str());
          break;
        }
      }
    }

    FD_ZERO(&read_fds);
    max_fd = 0;
    int connected_clients = 0;

    for(int i=0; i<NODE_MAXCOUNT; i++){
      if(clients[i].connected()){
        int sock_fd = clients[i].fd();
        if(sock_fd != -1) {
          FD_SET(sock_fd, &read_fds);
          if(sock_fd > max_fd) max_fd = sock_fd;
          connected_clients++;
        }
      } else {
          // If client not connected, try to stop it to ensure resources are released (even if already stopped)
          clients[i].stop();
      }
    }

    tv.tv_sec = 0;
    tv.tv_usec = 10; // 等待100微秒

    bool data_processed_in_this_iteration = false;

    if (max_fd > 0) {
      int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);

      if (ret < 0) {
        // Serial.printf("select error: %d\n", ret); // 频繁打印可能影响性能，仅在调试时开启
      } else if (ret > 0) { // 有数据可读
        for(int i=0; i<NODE_MAXCOUNT; i++){
          if(clients[i].connected()){
            int sock_fd = clients[i].fd();
            if(sock_fd != -1 && FD_ISSET(sock_fd, &read_fds)){
              size_t avail = clients[i].available();
              if(avail > 0){
                size_t readLen = clients[i].read(netBuffer[i] + netBufLen[i],
                                 min(avail, MAX_BUFFER_SIZE - netBufLen[i]));
                netBufLen[i] += readLen;
                data_processed_in_this_iteration = true;

                // 新增：为当前节点在当前select循环中处理的包数计数
                int packets_processed_this_node_in_cycle = 0; 

                while(netBufLen[i] >= 3 && packets_processed_this_node_in_cycle < MAX_PACKETS_PER_NODE_PER_ITERATION) {
                   uint16_t pktLen = (netBuffer[i][0] << 8) | netBuffer[i][1];
                   size_t totalPktSize = 2 + 1 + pktLen;

                   if(netBufLen[i] >= totalPktSize) {
                      if(pktLen == sizeof(NodeData)) {
                          uint8_t nodeID = netBuffer[i][2];
                          // processData不再检查缓冲区是否满，由sendTimerCallback中的提交逻辑决定
                          processData(nodeID, &netBuffer[i][3]);
                          packets_processed_this_node_in_cycle++; // 成功处理一个包，计数器加一
                      } else {
                          Serial.printf("Node %d: Invalid packet length received: %d, expected %d\n", i, pktLen, sizeof(NodeData));
                      }
                      size_t remain = netBufLen[i] - totalPktSize;
                      memmove(netBuffer[i], netBuffer[i]+totalPktSize, remain);
                      netBufLen[i] = remain;
                   } else {
                       break;
                   }
                }
                if(netBufLen[i] >= MAX_BUFFER_SIZE) {
                   Serial.printf("Node %d: Buffer overflow, clearing buffer.\n", i);
                   netBufLen[i] = 0;
                }
              }
            }
          }
        }
      }
    }

    // 检查连接是否断开
    for(int i=0; i<NODE_MAXCOUNT; i++){
        if(clients[i].connected() && clients[i].available() <= 0 && clients[i].fd() == -1) {
            Serial.printf("Client %d disconnected\n", i);
            clients[i].stop();
        }
    }

    if (!data_processed_in_this_iteration && connected_clients == 0) {
        vTaskDelay(pdMS_TO_TICKS(1)); // 当没有数据且没有客户端连接时，才稍微延迟
    }
    // 否则，select()的微秒超时已经提供了足够的让步，或者有数据需要立即处理
  }
}

// 串口接收任务函数 (改为USB CDC接收)
void taskUSBCommand(void *pvParam) {
  const TickType_t xDelay = pdMS_TO_TICKS(10); // 10ms
  static unsigned long lastCmdTime = 0; // 用于超时处理

  while(1) {
    size_t available = Serial.available();

    if(available > 0) {
      uint8_t tempBuf[CMD_FRAME_SIZE * 2]; // 临时缓冲区，稍大一些
      int readLen = Serial.readBytes(tempBuf, min(available, sizeof(tempBuf)));

      for(int i=0; i<readLen; i++) {
        // 简单按字节填充到cmdBuffer (无协议校验)
        if(cmdIndex < CMD_FRAME_SIZE) {
          cmdBuffer[cmdIndex++] = tempBuf[i];

          // 完整接收后立即转发
          if(cmdIndex == CMD_FRAME_SIZE) {
            // 广播给所有已连接节点
            for(int j=0; j<NODE_MAXCOUNT; j++) {
              if(clients[j].connected()) {
                clients[j].write(cmdBuffer, CMD_FRAME_SIZE);
                // clients[j].flush();  // TCP/IP 会有自己的缓冲，不一定需要立即flush，可能影响效率
              }
            }
            cmdIndex = 0;  // 重置接收状态
            lastCmdTime = millis(); // 更新接收时间
          }
        } else {
           // 如果 cmdIndex >= CMD_FRAME_SIZE 但还没处理完，说明有错误
           // 清空缓冲区并重置
           cmdIndex = 0;
           // 可以选择打印错误信息
           // Serial.println("Command buffer overflow/sync error");
        }
      }
      lastCmdTime = millis(); // 只要有数据进来就更新时间
    } else {
      // 检查命令接收超时
      if (cmdIndex > 0 && millis() - lastCmdTime > 100) { // 假设100ms没收到完整命令就认为超时错误
        Serial.printf("Command timeout, clearing buffer (index=%d)\n", cmdIndex);
        cmdIndex = 0;
      }
    }
    vTaskDelay(xDelay);
  }
}

// 新增发送任务
// void taskSendData(void *pvParam) {
//   static uint32_t lastSend = micros();
//   while(1) {
//     if(micros() - lastSend >= 500) { // 1ms
//       sendCombinedData();
//       lastSend = micros();
//     }
//     vTaskDelay(pdMS_TO_TICKS(1)); // 或更短的延迟，例如 0 或 1
//                                   // 使用 vTaskDelay(1) 是为了让 FreeRTOS 调度器有机会切换任务
//                                   // 如果需要极低延迟，可以考虑 vTaskDelay(0) 并在需要时让出CPU
//                                   // 但 vTaskDelay(1) 通常更稳定
//   }
// }


void setup() {
  // 初始化 USB CDC (Serial)，波特率参数对于USB是虚拟的，但通常需要一个
  Serial.begin(115200); // 或者使用任意值，如 460800


    // 初始化 activeCollectionSlotData
  activeCollectionSlotData.readyFlags.reset();
  // 确保所有 timeSlots 初始都是空的
  for (int i = 0; i < BUFFER_SLOTS; ++i) {
      timeSlots[i].readyFlags.reset();
  }
  // 启动网络任务 (核心1)
  xTaskCreatePinnedToCore(taskNetwork, "net", 8192, NULL, 5, NULL, 1); // 增加栈空间
  // 启动USB CDC指令处理任务 (核心0)
  xTaskCreatePinnedToCore(taskUSBCommand, "usb_cmd", 4096, NULL, 4, NULL, 0);
  // 启动数据发送任务 (核心0或1均可，优先级高)
  // xTaskCreatePinnedToCore(taskSendData, "send_data", 4096, NULL, 6, NULL, 0); // 优先级高于其他任务
 // 配置和启动发送定时器 (每1毫秒触发一次，即1KHz)
  // const esp_timer_create_args_t timer_args = {
  //     .callback = &sendTimerCallback,
  //     .name = "send_timer",
  //     .dispatch_method = ESP_TIMER_TASK // 确保回调在专用高优先级任务中执行
  // };
  const esp_timer_create_args_t timer_args = {
    &sendTimerCallback,         // callback
    nullptr,                    // arg (如果你没有给回调传递参数)
    ESP_TIMER_TASK,             // dispatch_method
    "send_timer"                // name
    // 如果有 skip_unhandled_events，根据需要添加 true/false
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &send_timer_handle));
  ESP_ERROR_CHECK(esp_timer_start_periodic(send_timer_handle, 1000)); // 1000微秒 = 1毫秒


  WiFi.softAP(ssid, password, 6, 0, NODE_MAXCOUNT); // 指定信道6
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  server.begin();
  esp_wifi_set_ps(WIFI_PS_NONE);  // 关闭WiFi休眠

  Serial.println("ESP32-S3 AP started with USB CDC"); // 使用USB CDC输出调试信息
}


void loop() {
  static uint32_t lastSend = micros();

  // // 高优先级发送控制（1KHz）
  // if(micros() - lastSend >= 1000) { // 1ms
  //   sendCombinedData();
  //   lastSend = micros();
  // }

  // loop里不需要太多的工作，主要逻辑都在FreeRTOS任务里
  vTaskDelay(pdMS_TO_TICKS(1000)); // 避免loop空转
}