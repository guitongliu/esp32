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
localIP = gethostbyname(gethostname())
# ESP32 IP 地址：192.168.4.1
# 上位机 IP 地址：192.168.4.2
# TargetIP=localIP
# 数据相关
maxLen = 25000
maxLen_acc = 5000
CHANNEL_SIZE = 24
# NUM_CHANNELS = 8
# SHOW_CHANNELS = 8
# SHOW_CHANNEL = 8
NUM_CHANNELS = 2
SHOW_CHANNELS = 2
SHOW_CHANNEL = 2
channelValues = [[] for _ in range(NUM_CHANNELS)]  # udp获取的总数据 8行 x列 二维列表，每一列是某通道数据
channelValues_acc = [[] for _ in range(3)]  # 三轴加速度，三列

chx = [0 for _ in range(NUM_CHANNELS)]
# chx=0 #数据暂存
count = 0
# showdata=np.zeros((8,maxLen),dtype=np.int32)
# np_channelValues=np.zeros((8,1),dtype=np.int32)
showdata = np.zeros((2, maxLen), dtype=np.int32)
showdata_acc = np.zeros((3, maxLen_acc), dtype=np.float32)
np_channelValues = np.zeros((2, 1), dtype=np.int32)
np_channelValues_acc = np.zeros((3, 1), dtype=np.float32)

RecodeFlag = 0
# 滤波器设计
sample_freq = 16000
sample_interval = 1 / sample_freq  # 采样间隔
Wn = 2 * 450 / sample_freq
b, a = signal.butter(4, Wn, 'lowpass')
low = 2 * 45 / sample_freq
high = 2 * 55 / sample_freq
low1 = 2 * 95 / sample_freq
high1 = 2 * 105 / sample_freq
b1, a1 = signal.butter(3, [low, high], 'bandstop')
b2, a2 = signal.butter(3, [low1, high1], 'bandstop')


def FFT_FUN(fft_data):
    '''
    频谱分析
    :param fft_data:
    :return:
    freq_shift 频率
    fft_amp_shift 幅度
    '''
    N1 = len(fft_data)
    fft_data = fft(fft_data)
    fft_amp = np.array(np.abs(fft_data) / N1 * 2)  # 用于计算双边谱
    fft_amp[0] = 0.5 * fft_amp[0]
    fft_amp_shift = fftshift(fft_amp)  # 使用fftshift将信号的零频移动到中间
    list1_shift = np.array(range(0, N1))
    freq_shift = sample_freq * (list1_shift / N1) - sample_freq / 2  # 零频移动后的频率轴
    return freq_shift, fft_amp_shift


# class UdpInit(socket):
#     def __init__(self,ip,port):
#         self.udpReceiver=self.socket(AF_INET, SOCK_DGRAM)
#         self.udpReceiver.bind((ip, port))
#         self.udpReceiver.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

def UdpInit():
    global udpReceiver, udpReceiver2, udpReceiver3

    # udpReceiver 接收下位机回复指令（未用到）
    udpReceiver = socket(AF_INET, SOCK_DGRAM)
    # udpReceiver.bind((localIP, Cmd_RX_port))
    udpReceiver.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    # udpReceiver2 接收下位机发送的ADC数据
    udpReceiver2 = socket(AF_INET, SOCK_DGRAM)
    udpReceiver2.bind((localIP, Data_port))  # 接收需要绑定端口
    udpReceiver2.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    # udpReceiver3 给下位机发送指令
    udpReceiver3 = socket(AF_INET, SOCK_DGRAM)
    # udpReceiver2.bind((localIP, Cmd_TX_port))
    udpReceiver3.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    print("Listening on", localIP, ":", Cmd_RX_port)
    print("Listening on", localIP, ":", Data_port)
    # print(struct.pack('BB', tx[0],tx[1]))

    # tx=(0x55,0x56)
    # udpReceiver3.sendto(struct.pack('BB', tx[0],tx[1]), (TargetIP, Cmd_TX_port))
    pass


# 发送命令帧，帧结构见论文
def GetCmdBuffer(Cmd0, Cmd1):  # Cmd0:指令位， Cmd1:指令值
    cmd_bufferx = bytearray(cmd_buffer_empty)
    print("Setting CMD")
    cmd_bufferx[2] = Cmd0
    cmd_bufferx[4] = Cmd1
    udpReceiver3.sendto(cmd_bufferx, (TargetIP, Cmd_TX_port))  # 向下位机发送cmd_bufferx数据
    return cmd_bufferx


# def Record(channelValues):
#     channelValue=np.array(channelValues,dtype=np.int32)
#     with open(fileName, 'a', encoding='utf-8', newline='') as file_obj:
#         writer = csv.writer(file_obj.transpose())
#         # print(channelValues)
#         writer.writerows(channelValue)
#     pass

def Record(channelValues):
    with open(fileName, 'a', encoding='utf-8', newline='') as file_obj:
        writer = csv.writer(file_obj)
        writer.writerows(channelValues.transpose())
    pass


t = time.time()


# 从udp获取数据
def GetValue():
    global channelValues, count, t, channelValues_acc
    while True:

        # time.sleep(0.001)
        # Receive the packet
        packetData, senderAddress = udpReceiver2.recvfrom(3 * 2 * CHANNEL_SIZE + 3 * CHANNEL_SIZE)  # 接收3byte*8channel*每个channel60个点的数据
        # packetData_acc, senderAddress = udpReceiver2.recvfrom(3 * 2 * CHANNEL_SIZE)  # 接收acc数据
        # print(len(packetData))
        # print(packetData_acc)
        channelValue = struct.unpack('B' * (len(packetData)-3 * CHANNEL_SIZE), packetData[:3 * 2 * CHANNEL_SIZE])  # 按照字节的顺序解包，包的结构见论文p19图4-3
        # channelValue_acc = struct.unpack('f' * len(packetData), packetData[360:])
        # x = channelValue_acc[0]  # 第一个轴的 x
        # y = channelValue_acc[1]  # 第一个轴的 y
        # z = channelValue_acc[2]  # 第一个轴的 z
        # channelValues_acc[0].append(x)
        # channelValues_acc[1].append(y)
        # channelValues_acc[2].append(z)
        num_acc_points = len(packetData[3 * 2 * CHANNEL_SIZE:]) // 12

        count += 1
        if count >= 267:
            count = 0
            # print(time.time() - t)  # 观察采样率是否正确
            t = time.time()
        for mun in range(CHANNEL_SIZE):  # 60
            for ch in range(NUM_CHANNELS):  # 2
                chx[ch] = channelValue[mun * 6 + 3 * ch] << 16 | channelValue[mun * 6 + 3 * ch + 1] << 8 | channelValue[
                    mun * 6 + 3 * ch + 2]
                # 拆包 mun*24+3*ch是通道数据定位，期中mun是当前采样点编号，ch是通道编号
                if (chx[ch] > 0x7FFFFF):
                    chx[ch] = ((~(chx[ch]) & 0x00FFFFFF) + 1) * -1  # 如果>0x7FFFFFF说明采样值为负，编程有符号的32位整型数据
                # chx[ch]=randint(10,30)
                # chx[ch] = ch
                channelValues[ch].append(chx[ch])  # 将采样值放入总数据数组
        # 解包加速度数据（假设ACC数据有3个轴）
        for i in range(num_acc_points):
            acc_packet = packetData[3 * 2 * CHANNEL_SIZE + i * 12: 3 * 2 * CHANNEL_SIZE + (i + 1) * 12]
            acc_x, acc_y, acc_z = struct.unpack('fff', acc_packet)

            # print(acc_x, acc_y, acc_z)
            channelValues_acc[0].append(acc_x)  # x轴加速度
            channelValues_acc[1].append(acc_y)  # y轴加速度
            channelValues_acc[2].append(acc_z)  # z轴加速度
            # print(channelValues_acc[0])
            # if RecodeFlag==1:
            #     recordThread = threading.Thread(target=Record, args=(chx,))
            #     # receiveThread.setDaemon(True)
            #     recordThread.start()


# MainWindow创建GUI界面
class MainWindow(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('pyqtgraph作图')
        self.win = pg.GraphicsLayoutWidget(show=True)  # 画图窗口，调用pyqtgraph

        layout = QtWidgets.QGridLayout()  # window布局
        # layout.setSpacing(10)
        layout.setColumnStretch(0, 1)  # 窗口第一列：放交互按钮
        layout.setColumnStretch(1, 10)  # 窗口第二列：放绘图通道
        # layout.setColumnStretch(11, 20)

        # layout.setRowStretch(0,1)
        # layout.setRowStretch(1, 1)
        # layout.setRowStretch(2, 1)
        # layout.setRowStretch(3, 1)
        # layout.setRowStretch(4, 1)
        # layout.setRowStretch(5, 1)

        # 创建其他Qt控件
        StartBotton = QtWidgets.QPushButton("Start")  # 开始接收数据并画图按键
        StartBotton.clicked.connect(self.StartBotton)  # 连接到StartBotton()回调函数
        StopBotton = QtWidgets.QPushButton("Stop")  # 停止接收数据并画图按键
        StopBotton.clicked.connect(self.StopBotton)
        Start_layout = QtWidgets.QHBoxLayout()  # QHBoxlayout()按顺序横向排布按钮
        Start_layout.addWidget(StartBotton)
        Start_layout.addWidget(StopBotton)
        # layout.addWidget(StartBotton,0,0,1,1)
        # layout.addWidget(StopBotton,0,1,1,1)

        RecodeStartBotton = QtWidgets.QPushButton("Start Recod")  # 开始记录数据保存到csv
        RecodeStartBotton.clicked.connect(self.RecodeStartBotton)  # 绑定回调函数
        RecodeStopBotton = QtWidgets.QPushButton("Stop Recod")  # 停止记录数据
        RecodeStopBotton.clicked.connect(self.RecodeStopBotton)
        StartRecod_layout = QtWidgets.QHBoxLayout()
        StartRecod_layout.addWidget(RecodeStartBotton)
        StartRecod_layout.addWidget(RecodeStopBotton)
        # layout.addWidget(RecodeStartBotton, 1, 0, 1, 1)
        # layout.addWidget(RecodeStopBotton, 1, 1, 1, 1)

        FileName_lb = QtWidgets.QLabel('Save File Name:')  # 存储文件名设置
        self.FileName_le = QtWidgets.QLineEdit()  # 文本输入框
        self.FileName_le.setPlaceholderText('{}'.format("File Name"))  # 提示作用的文件名
        # self.FileName_le.setValidator(QtGui.QIntValidator())
        # self.FileName_le.editingFinished.connect(self.MaxLen)
        FileName_layout = QtWidgets.QHBoxLayout()
        FileName_layout.addWidget(FileName_lb)
        FileName_layout.addWidget(self.FileName_le)
        # layout.addWidget(MaxLen_lb, 2, 0, 1, 1)
        # layout.addWidget(self.MaxLen_le, 2, 1, 1, 1)

        OSR_lb = QtWidgets.QLabel('OSR')  # OSR设定下拉菜单
        self.OSR_cb = QtWidgets.QComboBox()
        self.OSR_cb.addItems(osr_str)  # 添加OSR可选类型
        self.OSR_cb.currentIndexChanged.connect(self.OSR_fun)  # 当列表选择发生变化，调用OSR_fun()函数
        self.OSR_cb.setCurrentIndex(0)  # 默认选项
        OSR_layout = QtWidgets.QHBoxLayout()
        OSR_layout.addWidget(OSR_lb)  # 添加label
        OSR_layout.addWidget(self.OSR_cb)  # 添加下拉菜单
        # layout.addWidget(OSR_lb, 3, 0, 1, 1)
        # layout.addWidget(self.OSR_cb, 3, 1, 1, 1)

        PGA_lb = QtWidgets.QLabel('PGA Gain')  # PGA设定下拉菜单，参见OSR设定
        self.PGA_cb = QtWidgets.QComboBox()
        self.PGA_cb.addItems(PGAGain)
        self.PGA_cb.currentIndexChanged.connect(self.PGA_fun)
        self.PGA_cb.setCurrentIndex(2)
        PGA_layout = QtWidgets.QHBoxLayout()
        PGA_layout.addWidget(PGA_lb)
        PGA_layout.addWidget(self.PGA_cb)
        # layout.addWidget(PGA_lb, 4, 0, 1, 1)
        # layout.addWidget(self.PGA_cb, 4, 1, 1, 1)

        Filter_lb = QtWidgets.QLabel('Filter –3-dB')  # DCblock filter设定，参见OSR设定
        self.Filter_cb = QtWidgets.QComboBox()
        self.Filter_cb.addItems(DCBlockFilter)
        self.Filter_cb.currentIndexChanged.connect(self.Filter_fun)
        self.Filter_cb.setCurrentIndex(4)
        Filter_layout = QtWidgets.QHBoxLayout()
        Filter_layout.addWidget(Filter_lb)
        Filter_layout.addWidget(self.Filter_cb)
        # layout.addWidget(Filter_lb, 5, 0, 1, 1)
        # layout.addWidget(self.Filter_cb, 5, 1, 1, 1)
        # layout.addLayout(OSR_layout, 0, 0, 1, 1)
        layout.addLayout(Start_layout, 0, 0, 1, 1)  # 将Start，Stop部分的控件加入窗口（0,0列，占用1,1）
        layout.addLayout(StartRecod_layout, 1, 0, 1, 1)
        layout.addLayout(FileName_layout, 2, 0, 1, 1)
        layout.addLayout(OSR_layout, 3, 0, 1, 1)
        layout.addLayout(PGA_layout, 4, 0, 1, 1)
        layout.addLayout(Filter_layout, 5, 0, 1, 1)

        # layout.addWidget(Start_layout, 0, 0, 1, 1)
        # layout.addWidget(StartRecod_layout, 0, 0, 1, 1)
        # layout.addWidget(MaxLen_layout, 0, 0, 1, 1)
        # layout.addWidget(PGA_layout, 0, 0, 1, 1)
        # layout.addWidget(Filter_layout, 0, 0, 1, 1)
        # # layout.addWidget(Filter_layout, 0, 0, 1, 1)

        # hbox = QtWidgets.QHBoxLayout()
        # hbox.addLayout(layout)
        # hbox.addWidget(self.win)
        layout.addWidget(self.win, 0, 1, 8, 3)  # 将画图窗口加入
        self.setLayout(layout)

        # self.px = self.win.addPlot(row=1, col=0)
        # self.curve = self.px.plot()
        self.px = [0 for _ in range(4)]  # 增加8个画图通道
        self.curve = [0 for _ in range(4)]  # 曲线实例
        for i in range(1):
            self.px[i] = self.win.addPlot(row=i, col=0)
            self.px[i].setYRange(-200000, 200000)  # 设置Y轴值的范围
            self.px[i].setXRange(0, maxLen)  # 设置X轴值的范围
            self.curve[i] = self.px[i].plot()  # 将曲线实例绘制到通道里
        for i in range(1, 4):
            self.px[i] = self.win.addPlot(row=i, col=0)
            self.px[i].setYRange(-1, 1)  # 设置Y轴值的范围
            self.px[i].setXRange(0, maxLen_acc)  # 设置X轴值的范围
            self.curve[i] = self.px[i].plot()  # 将曲线实例绘制到通道里

        #
        # 启动定时器，每隔20ms通知刷新一次数据
        self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self.updateData)
        self.timer.timeout.connect(self.UpdateData)  # 调用UpdateData函数
        self.timer.start(20)  # 20ms更新一下画图数据

    def MaxLen(self):
        global maxLen
        s = self.MaxLen_le.text()
        # print(s)
        # print(type(s))
        s = int(s)
        # print(s)
        # print(type(s))
        maxLen = s
        print(maxLen)

    def StartBotton(self):
        GetCmdBuffer(CmdDic["cmd_Start"], 0x11)
        print("cmd_Start")

    def StopBotton(self):
        GetCmdBuffer(CmdDic["cmd_Start"], 0x55)
        print("cmd_Stop")

    def RecodeStartBotton(self):
        global fileName, RecodeFlag
        s = self.FileName_le.text()
        if len(s) == 0:
            fileName = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime()) + '.csv'
        elif s.split('.')[-1] != 'csv':
            fileName = s + '.csv'
        self.FileName_le.setPlaceholderText(fileName)
        print(fileName)

        RecodeFlag = 1
        with open(fileName, 'w', encoding='utf-8', newline='') as file_obj:
            header = ['ch0', 'ch1', 'ch2', 'ch3', 'ch4', 'ch5', 'ch6', 'ch7']
            writer = csv.writer(file_obj)
            writer.writerow(header)
        print("RecodeStartBotton")

    def RecodeStopBotton(self):
        global RecodeFlag
        RecodeFlag = 0
        print("RecodeStopBotton")

    # OSR设定函数
    def OSR_fun(self):

        print(self.OSR_cb.currentIndex())
        print(GetCmdBuffer(CmdDic["cmd_Set_OSR"],
                           self.OSR_cb.currentIndex()).hex())  # 调用了GetCmdBuffer函数，发送CMD0:指令位+CMD1指令值
        # CmdDic：指令位字典; currentIndex()返回可选项列表的下表值，正好对应到下位机寄存器二进制表示，顺序参见毕业论文表p15(3-2)

    # PGA设定函数，参考OSR
    def PGA_fun(self):

        print(self.PGA_cb.currentIndex())
        print(GetCmdBuffer(CmdDic["cmd_Set_PGA"], self.PGA_cb.currentIndex()).hex())

    # DC block filter设定函数，参考OSR
    def Filter_fun(self):

        print(self.Filter_cb.currentIndex())
        print(GetCmdBuffer(CmdDic["cmd_Set_Filter"], self.Filter_cb.currentIndex()).hex())

    # 更新数据画图函数
    def UpdateData(self):

        global channelValues, showdata, np_channelValues, channelValues_acc, showdata_acc, np_channelValues_acc
        leng = len(channelValues[0])  # 获取总数据某通道采样点个数
        leng_acc = len(channelValues_acc[0])
        # print(leng)
        # print(leng_acc)
        if leng > 1:
            np_channelValues = np.c_[
                np_channelValues, np.array([channelValues[i][:leng] for i in range(NUM_CHANNELS)], dtype=np.int32)]
            # 将二维的接收数据列表channelValue列表转化为numpyarray，然后清空接收数据列表
            # np_channelValues = np.array([channelValues[i][:leng] for i in range(NUM_CHANNELS)], dtype=np.int32)
            if RecodeFlag == 1:
                Record(np_channelValues)  # 存储数据
            for data in channelValues:
                del data[0:leng]  # 删除接收数据列表
        if leng_acc > 1:
            # 处理加速度数据
            np_channelValues_acc = np.c_[
                np_channelValues_acc, np.array([channelValues_acc[i][:leng_acc] for i in range(3)], dtype=np.float32)]
            if RecodeFlag == 1:
                Record(np_channelValues_acc)  # 存储数据
            for data in channelValues_acc:
                del data[0:leng_acc]  # 清空已处理的加速度数据
            # print(len)
        if leng - 1 > maxLen:  # 接收数据超过maxLen
            showdata = np_channelValues[:, :maxLen]  # showdata显示maxlen长度的数据
            np_channelValues = np.delete(np_channelValues, slice(0, maxLen), axis=1)  # np_channelValues删除maxLen数据
        elif leng > 1:  # 滑动展示
            showdata[:, :1 - leng] = showdata[:, leng - 1:]  # 将showdata最后的maxlen-leng个数据移至最前
            showdata[:, 1 - leng:] = np_channelValues[:, :leng - 1]  # 将showdata最后的leng个数据用np_channelValues中的leng个新数据替代
            np_channelValues = np.delete(np_channelValues, slice(0, leng - 1), axis=1)  # 将np_channelValues中数据删除

        if leng_acc - 1 > maxLen_acc:  # 同样处理加速度数据
            showdata_acc = np_channelValues_acc[:, :maxLen_acc]
            np_channelValues_acc = np.delete(np_channelValues_acc, slice(0, maxLen_acc), axis=1)
        elif leng_acc > 1:
            showdata_acc[:, :1 - leng_acc] = showdata_acc[:, leng_acc - 1:]
            showdata_acc[:, 1 - leng_acc:] = np_channelValues_acc[:, :leng_acc - 1]
            np_channelValues_acc = np.delete(np_channelValues_acc, slice(0, leng_acc - 1), axis=1)

        # print(showdata.shape)
        # 展示前数字滤波
        # print(showdata_acc[0, :])
        self.curve[0].setData(showdata[0, :])  # 将showdata画图
        self.curve[1].setData(showdata_acc[0, :])  # 将showdata画图
        self.curve[2].setData(showdata_acc[1, :])  # 将showdata画图
        self.curve[3].setData(showdata_acc[2, :])  # 将showdata画图
        # for i in range(8):
        # for i in range(1):
        #     # showdatax0 = signal.filtfilt(b, a, showdata[i,:])
        #     # # showdatax0 = showdata[i, :]
        #     # showdatax1 = signal.filtfilt(b1, a1, showdatax0)
        #     # showdatax2 = signal.filtfilt(b2, a2, showdatax1)
        #     #
        #     # # curve[i].setData(showdata[i,:])
        #     # self.curve[i].setData(showdatax2)
        #
        #     self.curve[i].setData(showdata[i,:]) #将showdata画图

        # freq_shift, fft_amp_shift = FFT_FUN(showdatax2)
        # self.curve[i + 4].setData(freq_shift, fft_amp_shift)
        pass


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    UdpInit()
    # GetCmdBuffer(0,0)
    # SetOSR(0x01)
    receive_Cmd_Thread = threading.Thread(target=GetValue)
    receive_Cmd_Thread.start()

    main = MainWindow()
    main.show()
    app.exec_()
