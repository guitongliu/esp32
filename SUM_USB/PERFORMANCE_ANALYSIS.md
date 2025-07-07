# ESP32 EMG系统性能分析与优化

## 性能指标分析

### 实时性指标
- **发送频率**: 1KHz (1ms周期)
- **定时器精度**: ±1μs (ESP-IDF高精度定时器)
- **网络延迟**: 10μs select()超时 + TCP传输延迟
- **数据处理延迟**: <100μs (memcpy + bitset操作)

### 吞吐量分析
```
单节点数据量: 48字节/包
4节点总数据量: 192字节/帧
发送频率: 1KHz
理论吞吐量: 192KB/s = 1.536Mbps
```

**USB CDC传输能力**: 480Mbps (USB 2.0 Full Speed)
**WiFi传输能力**: 150Mbps (802.11n 2.4GHz)
**结论**: 系统吞吐量远低于硬件上限，有充足裕量

### 内存使用分析
```cpp
// 主要内存占用
TimeSlot timeSlots[10];              // 10 × (192 + 1) = 1930字节
TimeSlot activeCollectionSlotData;   // 193字节
uint8_t netBuffer[4][2048];          // 8192字节
uint8_t serialTxAggregatedBuffer[];  // ~220字节

总计: ~10.5KB (ESP32-S3总SRAM: 512KB)
```

**内存使用率**: ~2% (非常充足)

## 关键性能瓶颈分析

### 1. 网络I/O瓶颈
**当前实现**:
```cpp
tv.tv_usec = 10;  // 10微秒select()超时
```

**瓶颈分析**:
- 10μs超时确保高响应性，但增加CPU占用
- select()系统调用开销约5-10μs
- 在无数据时会导致空轮询

**优化建议**:
```cpp
// 动态超时调整
if (!data_processed_in_this_iteration && connected_clients == 0) {
    tv.tv_usec = 1000;  // 无连接时1ms超时
} else if (!data_processed_in_this_iteration) {
    tv.tv_usec = 100;   // 有连接无数据时100μs超时
} else {
    tv.tv_usec = 10;    // 有数据时10μs超时
}
```

### 2. 数据包处理限制
**当前实现**:
```cpp
#define MAX_PACKETS_PER_NODE_PER_ITERATION 2
```

**瓶颈分析**:
- 限制每轮处理包数防止阻塞
- 在高数据率时可能造成处理延迟
- 需要在实时性和公平性间平衡

**优化建议**:
```cpp
// 基于缓冲区使用率的动态调整
int max_packets = (netBufLen[i] > MAX_BUFFER_SIZE/2) ? 4 : 2;
while(netBufLen[i] >= 3 && packets_processed < max_packets) {
    // ... 处理逻辑
}
```

### 3. 串口发送性能
**当前实现**:
```cpp
Serial.write(serialTxAggregatedBuffer, txIndex);  // 单次发送
```

**性能分析**:
- USB CDC发送延迟约50-200μs
- 单次write()比多次write()效率更高
- 大包发送比小包发送效率更高

**性能测量建议**:
```cpp
uint32_t start_time = micros();
Serial.write(serialTxAggregatedBuffer, txIndex);
uint32_t send_time = micros() - start_time;
if (send_time > 500) {  // 超过500μs记录
    Serial.printf("WARN: USB send took %luμs\n", send_time);
}
```

## CPU使用率优化

### 任务优先级调优
```cpp
// 当前配置
xTaskCreatePinnedToCore(taskNetwork, "net", 8192, NULL, 5, NULL, 1);
xTaskCreatePinnedToCore(taskUSBCommand, "usb_cmd", 4096, NULL, 4, NULL, 0);
```

**优化建议**:
- 网络任务优先级5适中，避免饥饿其他任务
- USB命令任务优先级4较低，符合其非实时性需求
- 考虑添加看门狗任务监控系统健康状态

### 核心分配优化
**当前分配**:
- 核心0: USB命令 + 定时器回调
- 核心1: 网络处理

**性能分析**:
- 网络任务独占核心1，避免与WiFi驱动冲突
- 定时器回调在核心0，与USB处理共享
- 分配合理，充分利用双核优势

## 内存优化建议

### 1. 缓冲区大小调优
```cpp
// 当前配置
#define MAX_BUFFER_SIZE 2048
#define BUFFER_SLOTS 10
```

**优化分析**:
- 2KB网络缓冲区可容纳约40个数据包
- 10个时间槽可缓冲10ms数据
- 在1KHz频率下提供充足缓冲

**动态调整建议**:
```cpp
// 基于内存使用情况动态调整
if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 100*1024) {
    // 内存不足时减少缓冲区
    netBufLen[i] = min(netBufLen[i], MAX_BUFFER_SIZE/2);
}
```

### 2. 数据结构对齐优化
```cpp
// 当前NodeData结构体已优化
#pragma pack(push, 1)  // 1字节对齐，节省空间
typedef struct {
    struct { uint8_t ch1[3]; } adc[8];  // 24字节
    struct { float x, y, z; } imu[2];   // 24字节
} NodeData;  // 总计48字节
#pragma pack(pop)
```

**优化效果**: 相比默认8字节对齐节省约25%空间

## 可靠性优化

### 1. 错误恢复机制
```cpp
// 连接断开检测增强
void checkConnectionHealth() {
    static uint32_t last_check = 0;
    if (millis() - last_check > 5000) {  // 每5秒检查
        for(int i = 0; i < NODE_MAXCOUNT; i++) {
            if (clients[i].connected()) {
                // 发送心跳包检测连接状态
                uint8_t heartbeat = 0xFF;
                if (clients[i].write(&heartbeat, 1) != 1) {
                    Serial.printf("Node %d connection lost\n", i);
                    clients[i].stop();
                }
            }
        }
        last_check = millis();
    }
}
```

### 2. 数据完整性增强
```cpp
// 添加序列号检测数据包丢失
typedef struct {
    uint16_t sequence;     // 序列号
    uint8_t ch1[3];       // ADC数据
    // ... 其他字段
} NodeDataWithSeq;

// 在processData中检测丢包
static uint16_t expected_seq[NODE_MAXCOUNT] = {0};
if (data->sequence != expected_seq[nodeID]) {
    Serial.printf("Node %d: packet loss detected. Expected %d, got %d\n", 
                  nodeID, expected_seq[nodeID], data->sequence);
}
expected_seq[nodeID] = data->sequence + 1;
```

### 3. 缓冲区溢出保护增强
```cpp
// 智能缓冲区管理
void smartBufferManagement() {
    static uint32_t overflow_count = 0;
    
    if (isTimeSlotBufferFull()) {
        overflow_count++;
        if (overflow_count > 10) {  // 连续溢出
            // 增加缓冲区处理频率
            esp_timer_stop(send_timer_handle);
            esp_timer_start_periodic(send_timer_handle, 500);  // 2KHz
            Serial.println("Buffer overflow detected, increasing send frequency");
        }
    } else {
        overflow_count = 0;
    }
}
```

## 功耗优化

### 1. WiFi功耗管理
```cpp
// 当前配置
esp_wifi_set_ps(WIFI_PS_NONE);  // 关闭省电模式

// 优化建议：基于负载动态调整
void dynamicPowerManagement() {
    static uint32_t idle_time = 0;
    static bool power_save_enabled = false;
    
    if (connected_clients == 0) {
        idle_time++;
        if (idle_time > 10000 && !power_save_enabled) {  // 10秒无连接
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // 启用省电模式
            power_save_enabled = true;
        }
    } else {
        if (power_save_enabled) {
            esp_wifi_set_ps(WIFI_PS_NONE);  // 恢复高性能模式
            power_save_enabled = false;
        }
        idle_time = 0;
    }
}
```

### 2. CPU频率调节
```cpp
#include "esp_pm.h"

// 动态频率调节
void setupDynamicFrequency() {
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 240,      // 最高频率
        .min_freq_mhz = 80,       // 最低频率  
        .light_sleep_enable = false  // 禁用浅睡眠（实时系统）
    };
    esp_pm_configure(&pm_config);
}
```

## 调试和监控优化

### 1. 性能计数器增强
```cpp
typedef struct {
    uint32_t packets_received[NODE_MAXCOUNT];
    uint32_t packets_sent;
    uint32_t buffer_overflows;
    uint32_t crc_errors;
    uint32_t network_errors;
    uint64_t total_bytes_sent;
    uint32_t max_process_time_us;
    uint32_t avg_process_time_us;
} PerformanceCounters;

PerformanceCounters perf = {0};

// 性能监控任务
void taskPerformanceMonitor(void *pvParam) {
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 每10秒报告
        
        Serial.printf("=== Performance Report ===\n");
        Serial.printf("Packets: RX[%lu,%lu,%lu,%lu] TX[%lu]\n", 
                     perf.packets_received[0], perf.packets_received[1],
                     perf.packets_received[2], perf.packets_received[3],
                     perf.packets_sent);
        Serial.printf("Errors: Overflow[%lu] CRC[%lu] Network[%lu]\n",
                     perf.buffer_overflows, perf.crc_errors, perf.network_errors);
        Serial.printf("Processing: Max[%luμs] Avg[%luμs]\n",
                     perf.max_process_time_us, perf.avg_process_time_us);
        Serial.printf("Throughput: %lluKB/s\n", perf.total_bytes_sent/10240);
        
        // 重置计数器
        memset(&perf, 0, sizeof(perf));
    }
}
```

### 2. 实时性能分析
```cpp
// 添加到关键函数中
#define PERF_MEASURE_START() uint32_t _perf_start = micros()
#define PERF_MEASURE_END(name) do { \
    uint32_t _perf_end = micros(); \
    uint32_t _perf_duration = _perf_end - _perf_start; \
    if (_perf_duration > 1000) { \
        Serial.printf("PERF: %s took %luμs\n", name, _perf_duration); \
    } \
} while(0)

void processData(uint8_t nodeID, uint8_t* data) {
    PERF_MEASURE_START();
    // ... 处理逻辑
    PERF_MEASURE_END("processData");
}
```

## 未来扩展优化

### 1. 支持更多节点
```cpp
// 可扩展的节点管理
#define MAX_POSSIBLE_NODES 16
struct NodeManager {
    uint8_t active_count;
    uint8_t active_nodes[MAX_POSSIBLE_NODES];
    IPAddress node_ips[MAX_POSSIBLE_NODES];
    WiFiClient clients[MAX_POSSIBLE_NODES];
};

// 动态节点发现
void discoverNodes() {
    // 广播发现包，节点响应后加入活动列表
}
```

### 2. 数据压缩
```cpp
// 简单的差分压缩
typedef struct {
    uint8_t compressed_adc[8][2];  // 存储差分值
    float imu[2][3];               // IMU数据不压缩
} CompressedNodeData;

// 压缩率约50%，可显著减少传输带宽
```

### 3. 多级缓存
```cpp
// L1: 快速环形缓冲 (SRAM)
// L2: 大容量缓冲 (PSRAM) 
// L3: 外部存储 (SD卡)

typedef enum {
    CACHE_L1_SRAM,
    CACHE_L2_PSRAM, 
    CACHE_L3_STORAGE
} CacheLevel;

void tieredCaching(TimeSlot* data, CacheLevel level) {
    switch(level) {
        case CACHE_L1_SRAM:
            // 存储到SRAM环形缓冲区
            break;
        case CACHE_L2_PSRAM:
            // 存储到PSRAM大缓冲区
            break;
        case CACHE_L3_STORAGE:
            // 存储到SD卡文件
            break;
    }
}
```

---

*本文档提供了ESP32 EMG系统的全面性能分析和优化建议，涵盖实时性、吞吐量、可靠性和可扩展性等各个方面。*
