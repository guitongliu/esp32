import serial
import csv
from PyQt5 import QtWidgets, QtGui
from pyqtgraph.Qt import QtCore
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import sys
from socket import *
import struct
import time
import numpy as np
import threading
from scipy import signal
from scipy.fftpack import fft, fftshift
from random import randint


# **** MODIFICATION START: 设置pyqtgraph的全局样式 ****
pg.setConfigOption('background', 'w') # 将背景设置为白色 ('w' 是 white 的简写)
pg.setConfigOption('foreground', 'k') # 将前景（坐标轴、文本等）设置为黑色 ('k' 是 black 的简写)
# 使用deque替代list，如果列表长度限制+固定chunk处理不够
from collections import deque
import queue  # **** MOD: 导入 queue 用于记录线程 ****

# 串口设置，替换为你的串口号（例如COM3或/dev/ttyUSB0）
# 注意：如果你是通过USB转串口接收数据，确保是正确的串口号和波特率
# pg.setConfigOptions(antialias=False)
try:
    ser = serial.Serial('COM3', 115200, timeout=0.1)  # Added timeout for robustness
    print(f"Successfully opened serial port COM3")
except serial.SerialException as e:
    print(f"Error opening serial port COM3: {e}")
    print("Please check if the port is correct and not in use.")
    # Exit gracefully or disable serial reading functionality
    ser = None  # Indicate serial is not available

# 通讯协议
head1 = 0x56
head2 = 0x55
End1 = 0xe6
End2 = 0xe5
CmdDic = {"cmd_Start": 0x11, "cmd_Set_OSR": 0x22, "cmd_Set_Filter": 0x33, "cmd_Set_PGA": 0x44}
cmd_Start = 0x11
cmd_Set_OSR = 0x22
cmd_Set_Filter = 0x33
cmd_Set_PGA = 0x44
cmd_buffer_empty = bytes([head1, head2, 0x00, 0x00, 0x00, 0x00, End1, End2])  # 通讯协议结构
cmd_bufferx = b''

PGAGain = ["PGA_GAIN_1", "PGA_GAIN_2", "PGA_GAIN_4", "PGA_GAIN_8", "PGA_GAIN_16", "PGA_GAIN_32", "PGA_GAIN_64",
           "PGA_GAIN_128", ]

osr_str = ["OSR_128", "OSR_256", "OSR_512", "OSR_1024", "OSR_2048", "OSR_4096", "OSR_8192", "OSR_16384", ]

DCBlockFilter = ["disabled", "181 Hz", "84.8 Hz", "41.1 Hz", "20.2 Hz", "10.0 Hz", "4.99 Hz", "2.49 Hz", "1.24 Hz",
                 "622 mHz", "311 mHz", "155 mHz", "77.7 mHz", "38.9 mHz", "19.4 mHz", "9.70 mHz"]

TargetIP = '192.168.4.1'  # ESP32作为热点的固定IP地址
# ESP32程序里定义的数据端口，指令端口
Data_port = 12345  # 数据端口号
Cmd_RX_port = 666  # 指令端口号
Cmd_TX_port = 667  # 指令端口号
# localIP = gethostbyname(gethostname()) # Get local IP dynamically if needed

# 数据相关
maxLen = 2500  # 绘图窗口显示的样本数（EMG）
maxLen_acc = 500  # 绘图窗口显示的样本数（ACC）
CHANNEL_SIZE = 8  # 一个数据包中EMG采样点的数量
CHANNEL_SIZE_ACC = 2

# 用户期望接收的节点数，根据这个来动态生成界面
node_expected = 4
# NUM_CHANNELS = 3
# 定义显示节点的列数
NODE_COLS = 2  # 期望将节点分为几列显示

# 数据累积列表
# These lists store raw data samples received from the serial port
# sized based on node_expected.
# channelValues[i] stores EMG channel 0 data for node i

deque_maxlen_emg = maxLen * 2  # 保留比显示内容稍多一些
deque_maxlen_acc = maxLen_acc * 2  # 对于ACC也一样

# channelValues = [[] for _ in range(node_expected)]
# # channelValues_acc[i*3 + axis] stores ACCel 1 axis (0=X, 1=Y, 2=Z) data for node i
# channelValues_acc = [[] for _ in range(node_expected * 3)]

channelValues = [deque(maxlen=deque_maxlen_emg) for _ in range(node_expected)]
# ^^^^ MOD: 每个EMG通道一个deque: node_expected * CHANNEL_SIZE
channelValues_acc = [deque(maxlen=deque_maxlen_acc) for _ in range(node_expected * 3)]
# ^^^^ MOD: 每个ACC轴一个deque: node_expected * CHANNEL_SIZE_ACC * 3 (轴)

# showdata 缓冲区，用于绘图，大小固定为 maxLen/maxLen_acc
# showdata[i, :] is EMG channel 0 data for node i
showdata = np.zeros((node_expected, maxLen), dtype=np.int32)
# showdata_acc[i*3 + axis, :] is ACCel 1 axis data for node i
showdata_acc = np.zeros((node_expected * 3, maxLen_acc), dtype=np.float32)

RecordFlag = 0  # 记录标志

data_lock = threading.Lock()  # 线程锁，用于保护数据列表的访问

# **** MOD: 记录线程设置 ****
record_queue = queue.Queue(maxsize=200)  # 用于待写入文件的数据块队列
recorder_thread_stop_event = threading.Event()


# def Record(channelValues):
#     with open(fileName, 'a', encoding='utf-8', newline='') as file_obj:
#         writer = csv.writer(file_obj)
#         writer.writerows(channelValues.transpose())
#     pass


def csv_writer_thread_func():
    global fileName  # 确保 fileName 可访问
    print("CSV写入线程已启动。")
    current_file_obj = None
    writer = None

    while not recorder_thread_stop_event.is_set():
        try:
            # 带超时等待数据，以便我们可以检查 stop_event
            # record_item 格式: (command, data_chunk_tuple, file_path)
            # data_chunk_tuple: (emg_data_np, acc_data_np)
            record_item = record_queue.get(timeout=0.1)
            command, data_tuple, file_path_for_item = record_item
            # print(data_tuple)

            if command == "STOP":
                if current_file_obj:
                    current_file_obj.close()
                    current_file_obj = None
                    writer = None
                    print(f"CSV写入器：已关闭文件 {fileName}")
                continue

            if command == "START":
                if current_file_obj and file_path_for_item != fileName:  # 如果文件名改变
                    current_file_obj.close()
                    current_file_obj = None  # 强制重新打开

                fileName = file_path_for_item  # 如果通过START更改，则更新全局fileName

                if not current_file_obj:  # 如果未打开或已关闭
                    try:
                        # RecordStartBotton 会处理 'w' 和表头
                        # 此处线程总是以 'a' (追加)模式打开
                        current_file_obj = open(fileName, 'a', encoding='utf-8', newline='')
                        writer = csv.writer(current_file_obj)
                        print(f"CSV写入器：已打开/追加到 {fileName}")
                    except IOError as e:
                        print(f"CSV写入线程：打开文件 {fileName} 错误: {e}")
                        current_file_obj = None
                        writer = None
                        continue
            # print(writer)
            if writer and data_tuple:
                emg_data_np, acc_data_np = data_tuple
                # emg_data_np = data_tuple
                # print(emg_data_np)
                # 假设 emg_data_np 和 acc_data_np 具有相同数量的样本（列）
                # 并且行对应于通道
                if emg_data_np is not None and emg_data_np.size > 0 and \
                        acc_data_np is not None and acc_data_np.size > 0:
                    # if emg_data_np is not None and emg_data_np.size > 0:

                    # num_samples = emg_data_np.shape[1]
                    # if acc_data_np.shape[1] != num_samples:
                    #     # print("CSV Warning: EMG and ACC sample count mismatch for recording.")
                    #     # 记录较少者，或跳过，或补齐 - 当前简单跳过不匹配的批次
                    #     min_s = min(emg_data_np.shape[1], acc_data_np.shape[1])
                    #     if min_s == 0: continue
                    #     emg_data_np = emg_data_np[:, :min_s]
                    #     acc_data_np = acc_data_np[:, :min_s]
                    #     num_samples = min_s
                    emg_samples = emg_data_np.shape[1]
                    acc_samples = acc_data_np.shape[1]
                    if emg_samples == 0: continue
                    # print(num_samples)
                    emg_data_np = emg_data_np[:, :emg_samples]
                    acc_data_np = acc_data_np[:, :acc_samples]

                    # 将EMG和ACC数据合并成一行进行写入
                    # 表头顺序：N0_EMG0..N0_EMG7, N0_ACC0X,Y,Z, N0_ACC1X,Y,Z, N1_EMG0...
                    # 这需要 data_tuple 包含所有节点的所有通道数据，并且顺序正确
                    # emg_data_np: (node_expected * CHANNEL_SIZE, num_samples)
                    # acc_data_np: (node_expected * CHANNEL_SIZE_ACC * 3, num_samples)

                    combined_data_for_rows = []  # row是行
                    emg_data_for_rows = []  # row是行
                    acc_data_for_rows = []  # row是行
                    # for sample_idx in range(emg_samples): #8
                    for n_idx in range(node_expected):  # 4
                        row_emgdata = []
                        row_accdata = []
                        # # 展平所有EMG通道的当前样本
                        # row_data.append(emg_data_np[:, sample_idx])
                        # # 展平所有ACC通道的当前样本
                        # # row_data.extend(acc_data_np[:, sample_idx])
                        # 展平所有EMG通道的当前样本 (前 node_expected 列)
                        # emg_data_np 形状是 (node_expected, 1)
                        # for n_idx in range(node_expected): #4
                        for sample_idx in range(emg_samples):  # 8
                            row_emgdata.append(emg_data_np[n_idx, sample_idx])
                        emg_data_for_rows.append(row_emgdata)
                        for sample_idx in range(acc_samples):  # 2
                            row_accdata.append(acc_data_np[n_idx * 3: n_idx * 3 + 3, sample_idx])
                        acc_data_for_rows.append(row_accdata)

                        # 展平所有ACC通道的当前样本 (接下来的 node_expected * 3 列)
                        # acc_data_np 形状是 (node_expected * 3, 1)
                    # for sample_idx in range(acc_samples):
                    # for n_idx in range(node_expected):  # 4
                    #     # for n_idx in range(node_expected):

                    if emg_data_for_rows:
                        writer.writerows(emg_data_for_rows)
                    if acc_data_for_rows:
                        writer.writerows(acc_data_for_rows)
                        # print("YES")

        except queue.Empty:
            continue  # 超时，检查 stop_event 并循环
        except Exception as e:
            print(f"CSV写入线程错误: {e}")
            if current_file_obj:
                try:
                    current_file_obj.close()
                except:
                    pass
                current_file_obj = None
            writer = None


# 发送命令帧，帧结构见论文
def GetCmdBuffer(Cmd0, Cmd1):  # Cmd0:指令位， Cmd1:指令值
    if ser is None or not ser.isOpen():
        print("Serial port not open, cannot send command.")
        return None

    cmd_bufferx = bytearray(cmd_buffer_empty)
    print(f"Setting CMD: Cmd0={Cmd0:#02x}, Cmd1={Cmd1:#02x}")
    cmd_bufferx[2] = Cmd0
    cmd_bufferx[4] = Cmd1
    try:
        ser.write(cmd_bufferx)
        print(f"Sent command: {cmd_bufferx.hex()}")
        # udpReceiver3.sendto(cmd_bufferx, (TargetIP, Cmd_TX_port))  # 向下位机发送cmd_bufferx数据
        return cmd_bufferx
    except serial.SerialException as e:
        print(f"Error sending command via serial: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred while sending command: {e}")
        return None


fileName = ""  # Initialize fileName globally


def crc16(data: bytes) -> int:
    """Calculates CRC16 (Polynomial 0x1021, Initial 0xFFFF)"""
    crc = 0xFFFF
    poly = 0x1021
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --- Data Reception Thread ---
# This function runs in a separate thread to read data from the serial port
# and parse the packets.
buffer = bytearray()  # Buffer to store incoming bytes
serial_rx_buffer = bytearray()  # **** MOD: 重命名以示区分 ****


def GetValue():
    global buffer, channelValues, channelValues_acc

    # Define max samples to keep in lists before UpdateData processes them
    # This limit should be larger than the plotting window maxLen to avoid visual glitches
    # but small enough to keep list operations fast.
    max_list_buffer_samples = maxLen * 5  # Keep up to 5 times the plot window size
    max_list_buffer_samples_acc = maxLen_acc * 5

    while True:
        if len(buffer) > 1024 * 1024:  # 如果缓冲区大小超过 1MB，清空缓冲区
            print("Buffer overflow, clearing...")
            buffer = bytearray()

        # === 强制对齐到包头 ===
        # Keep reading until a header is found or buffer gets too large
        while True:
            # Try reading some data first to avoid busy waiting if buffer is small
            if ser.in_waiting > 0:
                buffer.extend(ser.read(min(ser.in_waiting, 4096)))  # Read up to 4096 bytes
            elif len(buffer) < 4:  # If buffer too small and nothing waiting, wait for more data
                buffer.extend(ser.read(1024))  # Wait blocking or timeout

            pos = buffer.find(b'\xAA\x55')  # 严格匹配包头
            if pos != -1:
                # Discard data before header, but keep a few bytes potentially part of a split header
                del buffer[:max(0, pos - 3)]  # Keep last 3 bytes in case header was split
                break
            else:
                # If no header found and buffer is large, discard old data except the last few bytes
                if len(buffer) > 1024:  # If buffer is large and no header found
                    # print("Discarding old data in buffer, no header found")
                    buffer = buffer[-3:]  # Keep last few bytes and discard the rest
                    # Continue the loop to read more data

        # === 读取包头元数据 ===
        # Buffer should now start close to or at a header. Check if header is complete.
        if len(buffer) < 4:
            continue  # Not enough data for header, wait for more

        # Ensure header is exactly at the start after potential trimming
        if buffer[:2] != b'\xAA\x55':
            # This shouldn't happen if the previous loop logic is perfect, but as a safeguard
            # If it's not a header here, something is wrong, maybe discard a byte and try again?
            del buffer[0]
            continue

        header = buffer[:4]
        node_count = min(header[2], node_expected)  # 实际节点数
        # print(node_count)
        node_data_size = header[3]  # 每个节点的数据长度（应严格等于sizeof(NodeData)
        # print(node_data_size)

        # === 读取完整数据块（包含所有节点数据+CRC） ===
        # Data block size calculation: node_id (1 byte) + node_data_size per node + 2 bytes CRC
        expected_data_block_size = node_count * (1 + node_data_size) + 2
        # Total packet size = header (4 bytes) + data block (expected_data_block_size)
        total_packet_size = 4 + expected_data_block_size
        # print(total_packet_size)

        if len(buffer) < total_packet_size:
            # print(len(buffer))
            continue  # Not enough data for the full packet, wait for more

        # Extract the full packet for CRC and processing
        full_packet_bytes = buffer[:total_packet_size]
        del buffer[:total_packet_size]  # Remove the processed packet from the buffer

        # === CRC校验 ===
        # CRC is calculated over header + data (excluding the received CRC itself)
        data_for_crc = full_packet_bytes[:-2]  # Header + data payload
        received_crc_bytes = full_packet_bytes[-2:]

        try:
            calculated_crc = crc16(data_for_crc)
            received_crc = struct.unpack('>H', received_crc_bytes)[0]  # 大端解析
        except struct.error:
            print("CRC bytes unpack error")
            continue  # Skip this packet

        if calculated_crc != received_crc:
            # print(f"CRC error: Calculated {calculated_crc:04X}, Received {received_crc:04X}")
            continue  # Discard packet with CRC error

        # === Packet is valid, process data ===
        with data_lock:
            # Only process if we received the expected number of nodes
            if node_count == node_expected:
                ptr = 4  # Start processing after the 4-byte header

                # **** MODIFICATION START ****
                # 为当前数据包的EMG和ACC数据创建临时的NumPy数组，用于统一存储和队列发送
                # 形状为 (通道数, 1) 因为每次处理一个采样点
                current_packet_emg_samples_for_record = np.zeros((node_expected, 8), dtype=np.int32)
                current_packet_acc_samples_for_record = np.zeros((node_expected * 3, 2), dtype=np.float32)
                # **** MODIFICATION END ****

                received_nodes = []
                current_emg_samples = {}  # Temporarily store samples from this packet
                current_acc_samples = {}  # Temporarily store samples from this packet

                for _ in range(node_count):
                    if ptr + 1 + node_data_size > len(full_packet_bytes) - 2:  # Check bounds including CRC bytes at end
                        print("Packet parse error: unexpected end of data block")
                        break  # Malformed packet despite CRC? Skip processing this packet

                    node_id = full_packet_bytes[ptr]
                    ptr += 1

                    # Node ID validity check
                    if node_id >= node_expected:
                        print(f"Illegal Node ID: {node_id}")
                        ptr += node_data_size  # Skip data for this invalid node
                        continue

                    received_nodes.append(node_id)

                    # Extract and parse sensor data for this node
                    node_data = full_packet_bytes[ptr: ptr + node_data_size]
                    ptr += node_data_size

                    # Parse EMG data (4 channels)
                    # Ensure node_data has enough bytes for EMG
                    if len(node_data) < 3 * CHANNEL_SIZE:
                        print(f"Packet parse error: not enough EMG data for node {node_id}")
                        continue

                    # 解析EMG数据
                    emg_values = struct.unpack(f"{3 * CHANNEL_SIZE}B", node_data[:3 * CHANNEL_SIZE])
                    for sample in range(CHANNEL_SIZE):
                        byte_idx = sample * 3
                        ch_val = (emg_values[byte_idx] << 16 |
                                  emg_values[byte_idx + 1] << 8 |
                                  emg_values[byte_idx + 2])
                        # 符号扩展处理
                        if ch_val > 0x7FFFFF:
                            ch_val = ((~ch_val & 0x00FFFFFF) + 1) * -1

                            # -((~ch_val + 1) & 0x00FFFFFF)
                        channelValues[node_id].append(ch_val)
                        current_packet_emg_samples_for_record[node_id, sample] = ch_val  # 存储EMG0用于记录
                        # print(channelValues)

                    # Parse Accelerometer data (6 axes: 2 accelerometers?)
                    # Ensure node_data has enough bytes for ACC after EMG
                    acc_data_start = 3 * CHANNEL_SIZE
                    expected_acc_len = CHANNEL_SIZE_ACC * 12  # 6 sensors * 3 floats/sensor * 4 bytes/float
                    if len(node_data) < acc_data_start + expected_acc_len:
                        print(f"Packet parse error: not enough ACC data for node {node_id}")
                        continue

                    acc_data_bytes = node_data[acc_data_start: acc_data_start + expected_acc_len]

                    for i in range(CHANNEL_SIZE_ACC):
                        acc_x, acc_y, acc_z = struct.unpack('fff', acc_data_bytes[i * 12: (i + 1) * 12])
                        channelValues_acc[node_id * 3 + 0].append(acc_x)
                        channelValues_acc[node_id * 3 + 1].append(acc_y)
                        channelValues_acc[node_id * 3 + 2].append(acc_z)

                        current_packet_acc_samples_for_record[node_id * 3 + 0, i] = acc_x
                        current_packet_acc_samples_for_record[node_id * 3 + 1, i] = acc_y
                        current_packet_acc_samples_for_record[node_id * 3 + 2, i] = acc_z

                # **** MODIFICATION START ****
                # 如果记录标志为1，将处理好的当前数据包数据放入记录队列
                if RecordFlag:
                    try:
                        record_queue.put_nowait(("DATA", (
                            current_packet_emg_samples_for_record, current_packet_acc_samples_for_record),
                                                 fileName))
                    except queue.Full:
                        print("记录队列已满！GetValue数据丢失。")
                # **** MODIFICATION END ****

                # --- Manually enforce list size limit AFTER appending ---
                # This is necessary if not using deque with maxlen
                # for i in range(node_expected):
                #     if len(channelValues[i]) > max_list_buffer_samples:
                #         # Calculate how many old samples to trim
                #         trim_count = len(channelValues[i]) - max_list_buffer_samples
                #         del channelValues[i][:trim_count]  # Trim from the start
                #
                # for i in range(node_expected * 3):  # Iterate through ACC lists
                #     if len(channelValues_acc[i]) > max_list_buffer_samples_acc:
                #         trim_count = len(channelValues_acc[i]) - max_list_buffer_samples_acc
                #         del channelValues_acc[i][:trim_count]  # Trim from the start


# MainWindow创建GUI界面
class MainWindow(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f'肌电和加速度采集系统 ({node_expected} 节点)')  # Update window title
        self.win = pg.GraphicsLayoutWidget()  # 画图窗口，调用pyqtgraph
        self.win.ci.layout.setHorizontalSpacing(0)  # Reduce spacing between plots in the GLW grid
        self.win.ci.layout.setVerticalSpacing(0)  # Reduce spacing between plots in the GLW grid

        layout = QtWidgets.QGridLayout()  # window布局
        # layout.setSpacing(10)
        layout.setColumnStretch(0, 1)  # 窗口第一列：放交互按钮
        layout.setColumnStretch(1, 10)  # 窗口第二列：放绘图区域

        # --- Control Widgets ---
        StartBotton = QtWidgets.QPushButton("Start")  # 开始接收数据并画图按键
        StartBotton.clicked.connect(self.StartBotton)  # 连接到StartBotton()回调函数
        StopBotton = QtWidgets.QPushButton("Stop")  # 停止接收数据并画图按键
        StopBotton.clicked.connect(self.StopBotton)
        Start_layout = QtWidgets.QHBoxLayout()  # QHBoxlayout()按顺序横向排布按钮
        Start_layout.addWidget(StartBotton)
        Start_layout.addWidget(StopBotton)

        RecordStartBotton = QtWidgets.QPushButton("Start Record")  # 开始记录数据保存到csv
        RecordStartBotton.clicked.connect(self.RecordStartBotton)  # 绑定回调函数
        RecordStopBotton = QtWidgets.QPushButton("Stop Record")  # 停止记录数据
        RecordStopBotton.clicked.connect(self.RecordStopBotton)
        StartRecod_layout = QtWidgets.QHBoxLayout()
        StartRecod_layout.addWidget(RecordStartBotton)
        StartRecod_layout.addWidget(RecordStopBotton)

        FileName_lb = QtWidgets.QLabel('Save File Name:')  # 存储文件名设置
        self.FileName_le = QtWidgets.QLineEdit()  # 文本输入框
        self.FileName_le.setPlaceholderText('{}'.format("File Name"))  # 提示作用的文件名
        FileName_layout = QtWidgets.QHBoxLayout()
        FileName_layout.addWidget(FileName_lb)
        FileName_layout.addWidget(self.FileName_le)

        OSR_lb = QtWidgets.QLabel('OSR')  # OSR设定下拉菜单
        self.OSR_cb = QtWidgets.QComboBox()
        self.OSR_cb.addItems(osr_str)  # 添加OSR可选类型
        self.OSR_cb.currentIndexChanged.connect(self.OSR_fun)  # 当列表选择发生变化，调用OSR_fun()函数
        self.OSR_cb.setCurrentIndex(4)  # Default OSR_2048
        OSR_layout = QtWidgets.QHBoxLayout()
        OSR_layout.addWidget(OSR_lb)  # 添加label
        OSR_layout.addWidget(self.OSR_cb)  # 添加下拉菜单

        PGA_lb = QtWidgets.QLabel('PGA Gain')  # PGA设定下拉菜单，参见OSR设定
        self.PGA_cb = QtWidgets.QComboBox()
        self.PGA_cb.addItems(PGAGain)
        self.PGA_cb.currentIndexChanged.connect(self.PGA_fun)
        self.PGA_cb.setCurrentIndex(2)  # Default PGA_GAIN_4
        PGA_layout = QtWidgets.QHBoxLayout()
        PGA_layout.addWidget(PGA_lb)
        PGA_layout.addWidget(self.PGA_cb)

        Filter_lb = QtWidgets.QLabel('Filter –3-dB')  # DCblock filter设定，参见OSR设定
        self.Filter_cb = QtWidgets.QComboBox()
        self.Filter_cb.addItems(DCBlockFilter)
        self.Filter_cb.currentIndexChanged.connect(self.Filter_fun)
        self.Filter_cb.setCurrentIndex(4)  # Default 20.2 Hz
        Filter_layout = QtWidgets.QHBoxLayout()
        Filter_layout.addWidget(Filter_lb)
        Filter_layout.addWidget(self.Filter_cb)

        # Add control widgets to the left column (column 0)
        layout.addLayout(Start_layout, 0, 0, 1, 1)
        layout.addLayout(StartRecod_layout, 1, 0, 1, 1)
        layout.addLayout(FileName_layout, 2, 0, 1, 1)
        layout.addLayout(OSR_layout, 3, 0, 1, 1)
        layout.addLayout(PGA_layout, 4, 0, 1, 1)
        layout.addLayout(Filter_layout, 5, 0, 1, 1)
        # Add a spacer to push control widgets to the top
        layout.addItem(QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding), 6,
                       0)

        # --- Plotting Area ---
        # Add the GraphicsLayoutWidget to the right column (column 1)
        # It will span multiple rows to accommodate all plots
        # The row span should be large enough to contain all node rows * number of plots per node row
        # Number of plots per node column = 1 (EMG) + 3 (ACC) = 4
        # Number of node rows = ceil(node_expected / NODE_COLS)
        num_node_rows = (node_expected + NODE_COLS - 1) // NODE_COLS
        total_plot_rows_in_win = num_node_rows * 4  # 4 plots stacked per node in a column

        layout.addWidget(self.win, 0, 1, total_plot_rows_in_win, 1)  # Span calculated rows, 1 column in the main layout

        # --- Dynamically create plots based on node_expected ---
        self.plots = []  # List to hold PlotItem objects
        self.curves = []  # List to hold PlotDataItem objects

        # Create plots grouped by node
        for i in range(node_expected):
            # Calculate grid position within the GraphicsLayoutWidget
            node_col = i % NODE_COLS
            node_row_start = (i // NODE_COLS) * 4  # 4 rows per node (1 EMG + 3 ACC)

            # Add EMG plot for node i
            emg_plot = self.win.addPlot(row=node_row_start, col=node_col)
            emg_plot.setTitle(f'Node {i} EMG')
            # Only show X axis label on the bottom-most plot of the node stack
            if i // NODE_COLS == num_node_rows - 1 or NODE_COLS == 1 or i == node_expected - 1:
                # Show axis label if it's the last row of node plots, or only one node column, or the very last node
                # Let's simplify: only show on the absolute last row of plots
                if node_row_start + 3 == total_plot_rows_in_win - 1:  # Check if the last plot row is the last plot overall
                    emg_plot.setLabel('bottom', 'Samples')
                else:
                    # Remove X axis if not the last row of plots for this node column
                    # This is tricky with mixed EMG/ACC x-ranges. Let's keep X-axis for now, but maybe hide labels.
                    emg_plot.showAxis('bottom', show=True)  # Ensure axis is visible even without label
                    # emg_plot.getAxis('bottom').setStyle(showValues=False) # Hide values, keep ticks?
                    emg_plot.setLabel('bottom', '')  # Hide label
            else:
                emg_plot.setLabel('bottom', '')  # Hide label

            # Only show Y axis label on plots in the first column
            if node_col == 0:
                emg_plot.setLabel('left', 'Value (ADC)')
            else:
                emg_plot.setLabel('left', '')  # Hide Y label for other columns

            emg_plot.setYRange(-500000, 500000)  # Adjust Y range as needed
            emg_plot.setXLink(
                f'node{i}_x_axis')  # Link X axes within the node group - Use a unique name per node column
            emg_plot.setXRange(0, maxLen)
            emg_curve = emg_plot.plot(pen='k')
            self.plots.append(emg_plot)
            self.curves.append(emg_curve)

            # Add ACC plots for node i (X, Y, Z)
            acc_axes = ['X', 'Y', 'Z']
            for j in range(3):  # 3 ACC axes
                acc_plot = self.win.addPlot(row=node_row_start + j + 1, col=node_col)  # +1 because ACC starts after EMG
                acc_plot.setTitle(f'Node {i} Accel {acc_axes[j]}')

                # Only show X axis label on the bottom-most plot of the node stack
                # This will be the Accel Z plot for each node column
                if j == 2:  # This is the Accel Z plot (the last one in the stack for this node)
                    # Check if this is the actual last row of plots in the GLW
                    if node_row_start + j + 1 == total_plot_rows_in_win - 1:
                        acc_plot.setLabel('bottom', 'Samples')
                    else:
                        acc_plot.showAxis('bottom', show=True)
                        acc_plot.setLabel('bottom', '')  # Hide label

                else:  # Not the last plot in the node stack (AccX, AccY)
                    acc_plot.setLabel('bottom', '')  # Hide X label

                # Only show Y axis label on plots in the first column
                if node_col == 0:
                    acc_plot.setLabel('left', 'Value (g?)')
                else:
                    acc_plot.setLabel('left', '')  # Hide Y label for other columns

                acc_plot.setYRange(-2, 2)  # Adjust Y range for acceleration (e.g., +/- 2g)
                acc_plot.setXLink(f'node{i}_x_axis')  # Link X axes within the node group
                acc_plot.setXRange(0, maxLen_acc)  # ACC X range might be different from EMG maxLen
                acc_curve = acc_plot.plot(pen='k')
                self.plots.append(acc_plot)
                self.curves.append(acc_curve)

        self.setLayout(layout)

        # --- Timer for Plot Updates ---
        # Define the number of new samples to consume from the data lists per update tick.
        self.process_chunk_size_emg = 400  # Consume up to 100 EMG samples per list per tick
        self.process_chunk_size_acc = 80  # Consume up to 50 ACC samples per list per tick
        # Note: Data acquisition adds one EMG sample and one ACC sample (X, Y, Z) per node per packet.
        # So these chunk sizes refer to number of *packets* to process per update.
        # Let's use a single chunk size as data comes in packets.

        # 启动定时器，每隔20ms通知刷新一次数据
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.UpdateData)  # 调用UpdateData函数
        self.timer.start(20)  # 20ms更新一下画图数据 (50 Hz)

    def StartBotton(self):
        print("Start button clicked")
        GetCmdBuffer(CmdDic["cmd_Start"], 0x11)

    def StopBotton(self):
        print("Stop button clicked")
        GetCmdBuffer(CmdDic["cmd_Start"], 0x55)

    def RecordStartBotton(self):  # **** MOD: 修改以配合记录线程 ****
        global fileName, RecordFlag
        s = self.FileName_le.text()
        if not s:
            s = time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(time.time())) + '.csv'
        if not s.lower().endswith('.csv'):
            s += '.csv'
        fileName = s
        self.FileName_le.setText(fileName)
        print(f"尝试开始记录到 {fileName}")

        # 构建表头
        header = []
        for n_idx in range(node_expected):
            header.append(f'N{n_idx}_EMG')
            for axis in ['X', 'Y', 'Z']:
                header.append(f'N{n_idx}_IMU_{axis}')

        try:
            # 主线程创建/截断文件并写入表头
            with open(fileName, 'w', encoding='utf-8', newline='') as file_obj:
                writer = csv.writer(file_obj)
                writer.writerow(header)

            # 通知写入线程使用此文件（它将以追加模式打开）
            record_queue.put(("START", None, fileName))
            RecordFlag = 1
            print(f"记录已开始到 {fileName}。表头已写入。")

        except IOError as e:
            print(f"为写入表头打开文件 {fileName} 错误: {e}")
            QtWidgets.QMessageBox.warning(self, "记录错误", f"为写入表头打开文件错误:\n{e}")
            RecordFlag = 0
        except Exception as e:
            print(f"开始记录时发生意外错误: {e}")
            RecordFlag = 0

    def RecordStopBotton(self):  # **** MOD: 修改以配合记录线程 ****
        global RecordFlag
        if RecordFlag:
            RecordFlag = 0
            # 通知写入线程停止并关闭文件
            record_queue.put(("STOP", None, fileName))
            print(f"文件 {fileName} 的记录已停止。")
        else:
            print("记录未激活。")

    # OSR设定函数
    def OSR_fun(self):
        selectedIndex = self.OSR_cb.currentIndex()
        print(f"Selected OSR index: {selectedIndex}")
        GetCmdBuffer(CmdDic["cmd_Set_OSR"], selectedIndex)

    # PGA设定函数，参考OSR
    def PGA_fun(self):
        selectedIndex = self.PGA_cb.currentIndex()
        print(f"Selected PGA index: {selectedIndex}")
        GetCmdBuffer(CmdDic["cmd_Set_PGA"], selectedIndex)

    # DC block filter设定函数，参考OSR
    def Filter_fun(self):
        selectedIndex = self.Filter_cb.currentIndex()
        print(f"Selected Filter index: {selectedIndex}")
        GetCmdBuffer(CmdDic["cmd_Set_Filter"], selectedIndex)

    def UpdateData(self):
        global channelValues, showdata, channelValues_acc, showdata_acc, RecordFlag

        # Define how many samples (packets) to try and process this tick
        samples_to_process_acc = self.process_chunk_size_acc
        samples_to_process_emg = self.process_chunk_size_emg

        processed_emg_chunk = None
        processed_acc_chunk = None

        # 为绘图提取的数据块
        emg_data_for_plot_list = [[] for _ in range(node_expected)]
        acc_data_for_plot_list = [[] for _ in range(node_expected * 3)]

        with data_lock:

            # all_channel_lists = channelValues + channelValues_acc
            # min_len = min(len(ch) for ch in all_channel_lists) if all_channel_lists else 0
            # min_len_emg = min(len(ch) for ch in channelValues) if all_channel_lists else 0

            min_len_emg = min(len(d) for d in channelValues) if channelValues else 0
            num_to_pop_emg = min(min_len_emg, samples_to_process_emg)
            actual_samples_to_process_emg = num_to_pop_emg
            # print(actual_samples_to_process_emg)

            min_len_acc = min(len(d) for d in channelValues_acc) if channelValues_acc else 0
            num_to_pop_acc = min(min_len_acc, samples_to_process_acc)
            actual_samples_to_process = num_to_pop_acc
            # print(actual_samples_to_process)

            # actual_samples_to_process = min(samples_to_process_acc, min_len)
            # actual_samples_to_process_emg = min(samples_to_process_emg, min_len_emg)

            if actual_samples_to_process_emg > 0:
                # === Process EMG Data ===
                # # Extract the chunk from the start of each EMG list
                # emg_chunk_list = [ch[:actual_samples_to_process_emg] for ch in channelValues]
                # # Convert to numpy array (transpose so each row is a channel, columns are samples)
                # processed_emg_chunk = np.array(emg_chunk_list, dtype=np.int32)
                for i in range(node_expected):
                    for _ in range(num_to_pop_emg):
                        emg_data_for_plot_list[i].append(channelValues[i].popleft())
                emg_data_for_record_np = np.array(emg_data_for_plot_list, dtype=np.int32)

                # if RecordFlag == 1:
                #     Record(processed_emg_chunk)  # 存储数据

                # Delete processed samples from the lists (inefficient for lists, but size limited now)
                # for ch in channelValues:
                #     del ch[:actual_samples_to_process_emg]

                # Update the EMG plotting buffer (rolling)
                showdata = np.roll(showdata, -actual_samples_to_process_emg, axis=1)
                showdata[:, -actual_samples_to_process_emg:] = emg_data_for_record_np[:, :actual_samples_to_process_emg]

            if actual_samples_to_process > 0:

                # # === Process ACC Data ===
                # # Extract the chunk from the start of each ACC list
                # acc_chunk_list = [ch[:actual_samples_to_process] for ch in channelValues_acc]
                # # Convert to numpy array (transpose so each row is an acc channel, columns are samples)
                # processed_acc_chunk = np.array(acc_chunk_list, dtype=np.float32)
                for i in range(node_expected * 3):
                    for _ in range(num_to_pop_acc):
                        acc_data_for_plot_list[i].append(channelValues_acc[i].popleft())
                acc_data_for_record_np = np.array(acc_data_for_plot_list, dtype=np.float32)
                #
                # if RecordFlag == 1:
                #     Record(processed_acc_chunk)  # 存储数据

                # Delete processed samples from the lists
                # for ch in channelValues_acc:
                #     del ch[:actual_samples_to_process]

                # Update the ACC plotting buffer (rolling)
                showdata_acc = np.roll(showdata_acc, -actual_samples_to_process, axis=1)
                showdata_acc[:, -actual_samples_to_process:] = acc_data_for_record_np[:, :actual_samples_to_process]

                # === Record Data (assuming chunks are aligned) ===
                # You need to combine processed_emg_chunk and processed_acc_chunk for recording
                pass  # Data processed and added to showdata buffers


        for i in range(node_expected):
            self.curves[i * 4].setData(showdata[i, :])
            for j in range(3):
                plot_idx = i * 4 + j + 1
                self.curves[plot_idx].setData(showdata_acc[j + i * 3, :])
        #
        # for i in range(node_expected * 3):
        #     plot_idx = node_expected + i
        #     self.curves[plot_idx].setData(showdata_acc[i, :])

    def closeEvent(self, event):  # **** MOD: 清理关闭 ****
        print("正在关闭应用程序...")
        if ser and ser.isOpen():
            print("正在关闭串口。")
            ser.close()

        recorder_thread_stop_event.set()  # 通知写入线程停止
        if hasattr(self, 'recorder_thread_instance') and self.recorder_thread_instance.is_alive():
            if RecordFlag:  # 如果仍在录制，发送最终的STOP命令
                record_queue.put(("STOP", None, fileName))
            self.recorder_thread_instance.join(timeout=1)  # 等待写入线程完成
        print("应用程序已关闭。")
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    # Start data reception thread ONLY if serial port was opened successfully
    if ser is not None and ser.isOpen():
        print("Starting data reception thread...")
        receive_data_thread = threading.Thread(target=GetValue,
                                               daemon=True)  # daemon=True allows thread to exit with main app
        receive_data_thread.start()
    else:
        print("Serial port failed to open. Data reception thread not started.")

    main = MainWindow()
    # 启动CSV写入线程
    # **** MOD: 保存线程实例以便join ****
    main.recorder_thread_instance = threading.Thread(target=csv_writer_thread_func, daemon=True)
    main.recorder_thread_instance.start()

    main.show()
    sys.exit(app.exec_())  # Use sys.exit for proper application termination
