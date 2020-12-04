#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @File     : path.py
# @Project  : Sleipnir
# @Software : PyCharm
# @Author   : why
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/7/30 上午10:45
from displayGUI import *
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QMessageBox, QFileDialog,QLabel
import sys

from PyQt5.QtCore import QThread,QTimer,QRect,pyqtSignal,QRectF,Qt,QIODevice
from PyQt5.QtGui import QColor,QPainterPath,QFont,QPainter,QTextCursor
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,QMessageBox
from PyQt5.QtSerialPort import QSerialPortInfo,QSerialPort

import serial.tools.list_ports
import qdarkstyle

import sys

mySystem=sys.platform

class SwitchBtn(QWidget):
    #信号
    checkedChanged = pyqtSignal(bool)
    def __init__(self,parent=None):
        super(QWidget, self).__init__(parent)

        self.checked = False
        self.bgColorOff = QColor(255, 255, 255)
        self.bgColorOn = QColor(0, 0, 0)

        self.sliderColorOff = QColor(100, 100, 100)
        self.sliderColorOn = QColor(100, 184, 255)

        self.textColorOff = QColor(143, 143, 143)
        self.textColorOn = QColor(255, 255, 255)

        self.textOff = "OFF"
        self.textOn = "ON"

        self.space = 2
        self.rectRadius = 5

        self.step = self.width() / 50
        self.startX = 0
        self.endX = 0

        self.timer = QTimer(self)  # 初始化一个定时器
        self.timer.timeout.connect(self.updateValue)  # 计时结束调用operate()方法

        #self.timer.start(5)  # 设置计时间隔并启动

        self.setFont(QFont("Microsoft Yahei", 10))

        #self.resize(55,22)

    def updateValue(self):
        if self.checked:
            if self.startX < self.endX:
                self.startX = self.startX + self.step
            else:
                self.startX = self.endX
                self.timer.stop()
        else:
            if self.startX  > self.endX:
                self.startX = self.startX - self.step
            else:
                self.startX = self.endX
                self.timer.stop()

        self.update()


    def mousePressEvent(self,event):
        self.checked = not self.checked
        # 每次移动的步长为宽度的50分之一
        self.step = self.width() / 50
        #状态切换改变后自动计算终点坐标
        if self.checked:
            self.endX = self.width() - self.height()
        else:
            self.endX = 0
        self.timer.start(5)
        # 发射信号
        self.checkedChanged.emit(self.checked)

    def paintEvent(self, evt):
        #绘制准备工作, 启用反锯齿
            painter = QPainter()
            painter.begin(self)
            painter.setRenderHint(QPainter.Antialiasing)
            #绘制背景
            self.drawBg(evt, painter)
            #绘制滑块
            self.drawSlider(evt, painter)
            #绘制文字
            self.drawText(evt, painter)
            painter.end()

    def drawText(self, event, painter):
        painter.save()
        if self.checked:
            painter.setPen(self.textColorOn)
            painter.drawText(0, 0, self.width() / 2 + self.space * 2, self.height(), Qt.AlignCenter, self.textOn)
        else:
            painter.setPen(self.textColorOff)
            painter.drawText(self.width() / 2, 0,self.width() / 2 - self.space, self.height(), Qt.AlignCenter, self.textOff)
        painter.restore()


    def drawBg(self, event, painter):
        painter.save()
        painter.setPen(Qt.NoPen)
        if self.checked:
            painter.setBrush(self.bgColorOn)
        else:
            painter.setBrush(self.bgColorOff)
        rect = QRect(0, 0, self.width(), self.height())
        #半径为高度的一半
        radius = rect.height() / 2
        #圆的宽度为高度
        circleWidth = rect.height()
        path = QPainterPath()
        path.moveTo(radius, rect.left())
        path.arcTo(QRectF(rect.left(), rect.top(), circleWidth, circleWidth), 90, 180)
        path.lineTo(rect.width() - radius, rect.height())
        path.arcTo(QRectF(rect.width() - rect.height(), rect.top(), circleWidth, circleWidth), 270, 180)
        path.lineTo(radius, rect.top())
        painter.drawPath(path)
        painter.restore()

    def drawSlider(self, event, painter):
        painter.save()
        if self.checked:
            painter.setBrush(self.sliderColorOn)
        else:
            painter.setBrush(self.sliderColorOff)
        rect = QRect(0, 0, self.width(), self.height())
        sliderWidth = rect.height() - self.space * 2
        sliderRect = QRect(self.startX + self.space, self.space, sliderWidth, sliderWidth)
        painter.drawEllipse(sliderRect)
        painter.restore()

    def statusSwitch(self,status):
        self.checked=status
        # 每次移动的步长为宽度的50分之一
        self.step = self.width() / 50
        #状态切换改变后自动计算终点坐标
        if status:
            self.endX = self.width() - self.height()
        else:
            self.endX = 0
        self.timer.start(5)

class MyPort(QWidget):
    gpsDataSig=pyqtSignal(str)
    gpsCmdSig=pyqtSignal(str)
    gpsAvailablePortSig=pyqtSignal(list)
    portOpenSig=pyqtSignal(bool)
    def __init__(self,parent=None):
        super(MyPort, self).__init__(parent)
        self.ser=None
        self.portOpen=False
        self.headerList = ['GPHPD', 'GTIMU', 'GPFPD']
        self.portInfo=QSerialPortInfo()
        self.availablePorts=[]
        self.port=QSerialPort()
        self.paritySet={'even':QSerialPort.EvenParity,'odd':QSerialPort.OddParity,'none':QSerialPort.NoParity}
        self.time=QTimer()
        self.time.timeout.connect(self.updatePort)
        self.time.start(500)

    def closePort(self):
        self.portOpen=False
        self.port.close()

    def openPort(self,portConfig):
        global mySystem
        if mySystem=='win32' or mySystem=='win64':
            self.port.setPortName(self.portInfo.availablePorts()[portConfig['portIndex']].portName())
        elif mySystem=='linux':
            self.port.setPortName(self.portInfo.availablePorts()[portConfig['portIndex']].portName())
        else:
            return False
        if not self.port.setBaudRate(int(portConfig['baudrate']),QSerialPort.AllDirections):
            return False
        if not self.port.setStopBits(int(portConfig['stopbits'])):
            return False
        if not self.port.setParity(self.paritySet[portConfig['parity']]):
            return False
        if not self.port.open(QIODevice.ReadWrite):
            return False
        return True

    def readPort(self):
        if self.portOpen:  # 如果串口打开
            isCmd = True
            try:
                # rxData = bytes(self.port.readAll())
                # data=rxData.decode('UTF-8')
                if self.port.canReadLine():
                    data=str(self.port.readLine())
                    for header in self.headerList:  # 判断是否是数据
                        if header in data:
                            self.gpsDataSig.emit(data)  # 是数据发送GUI显示
                            isCmd = False
                    if '$' in data and isCmd:
                        self.gpsCmdSig.emit(data)  # 不是数据发送GUI窗口显示
            except:
                pass

    def updatePort(self):
        if len(self.availablePorts) != len(QSerialPortInfo().availablePorts()):#如果串口信息改变
            self.availablePorts = portInfo.availablePorts()  # 更新串口信息
            self.portOpen=False#串口标志位关闭
            self.port.close()#关闭串口
            availablePortList=[]
            for port in self.availablePorts:
                availablePortList.append(port.description()+' ('+port.portName()+')')
            self.gpsAvailablePortSig.emit(availablePortList)#发送新的串口信息
            self.portOpenSig.emit(self.portOpen)#GUI按钮状态改变

    # def writePort(self,data):
    #     if self.portOpen:
    #         if type(data) is str:
    #             self.port.write(data+'\n\r')

class Message(QLabel):
    def __init__(self,parent=None):
        super(Message,self).__init__(parent)
        self.color={'red':'255,0,0','green':'0,255,0','white':'255,255,255','black':'0,0,0','blue':'0,0,255','orange':'255,153,0'}
        self.setGeometry(QRect(0, 480, 981, 30))
        font = QFont()
        font.setPointSize(10)
        self.setFont(font)
        self.setObjectName("INS_GPS_message")
        self.time=QTimer()
        self.time.timeout.connect(self.gradients)
        self.transparent=0
        self.backgroundColor='0,0,0'
        self.textColor='0,0,0'
        self.Text=''

    def colorConstraint(self,color,Type):
        if type(color) is str:
            if color in self.color:
                if Type=='background':
                    self.backgroundColor = self.color[color]
                    return True
                elif Type=='text':
                    self.textColor=self.color[color]
                    return True
                else:
                    return False
            else:

                return False
        elif type(color) is list or type(color) is tuple:
            if  len(color) == 3 and max(color) <= 255 and min(color) >= 0:
                if Type=='background':
                    self.backgroundColor = str(color)[1:-1]
                    return True
                elif Type=='text':
                    self.textColor=str(color)[1:-1]
                    return True
                else:
                    return False
            else:
                return False

    def setStatusMessage(self,Text,backgroundColor,textColor):
        self.transparent=250
        self.setText(Text)
        self.time.start(50)
        if not self.colorConstraint(backgroundColor,'background'):
            raise KeyError('颜色设置错误！')
        if not self.colorConstraint(textColor,'text'):
            raise KeyError('颜色设置错误！')

    def gradients(self):
        if self.transparent>=0:
            self.setStyleSheet('background-color:'+'rgba('+self.backgroundColor+','+str(self.transparent)+
                               ');color: rgba('+self.textColor+','+str(self.transparent)+');')
            self.transparent-=10
        else:
            self.time.stop()

class MyWindows(QMainWindow,Ui_MainWindow,QWidget):
    def __init__(self,parent=None):
        super(MyWindows,self).__init__(parent)
        self.setupUi(self)
        self.retranslateUi(self)
        self.openPortBtn=SwitchBtn(self)
        self.openPortBtn.setGeometry(860,70,60,25)
        self.command={'navigationStatus':'$cmd,get,navmode*ff',
                      'COMConfigure':'$cmd,get,com*ff',
                      'COMOutput':'$cmd,get,output*ff',
                      'GNSSLeverarm':'$cmd,get,leverarm*ff',
                      'odometerStatus':'$cmd,get,pulse*ff',
                      'saveConfig':'$cmd,save,config*ff',
                      'QX':['$cmd,output,com2,null*ff','$cmd,output,com2,gpgga,1*ff','$cmd,save,config*ff'],
                      'radioStation':['$cmd,output,com2,null*ff','$cmd,save,config*ff']}

        self.rate2speed={'1Hz':'1','2Hz':'0.5','5Hz':'0.2','10Hz':'0.1','20Hz':'0.05','100Hz':'0.01','null':'Null','new':'New'}
        self.myPort=MyPort()
        self.INS_GPS_message=Message(self.centralwidget)
        self.setConnect()

    def setConnect(self):
        self.openPortBtn.checkedChanged.connect(self.switchPort)
        self.INS_port_select_.currentIndexChanged.connect(self.closePort)
        self.INS_inquiry_GNSS.clicked.connect(self.inquirtConfig)
        self.INS_inquiry_navigation_status.clicked.connect(self.inquirtConfig)
        self.INS_inquiry_odometer_status.clicked.connect(self.inquirtConfig)
        self.INS_inquiry_COM_configure.clicked.connect(self.inquirtConfig)
        self.INS_inquiry_COM_output.clicked.connect(self.inquirtConfig)
        self.INS_signal_source_QX.clicked.connect(self.modeConfig)
        self.INS_signal_source_radio_station.clicked.connect(self.modeConfig)
        self.INS_COM_protocol_config_confirm.clicked.connect(self.comProcotolConfig)
        self.INS_COM_output_config_confirm.clicked.connect(self.comOutputConfig)
        self.INS_COM_through_config_confirm.clicked.connect(self.comThroughConfig)
        self.INS_save_config.clicked.connect(self.saveConfig)
        self.INS_cmd_send_message_send.clicked.connect(self.cmdSendWindow)
        self.INS_information_receive_message_clear.clicked.connect(self.infoReceiveWindowClear)
        self.myPort.gpsAvailablePortSig.connect(self.updateAvailablePortSig)
        self.myPort.port.readyRead.connect(self.myPort.readPort)
        self.myPort.gpsCmdSig.connect(self.infoReceiveWindow)
        self.myPort.gpsDataSig.connect(self.displayGPSData)
        self.myPort.portOpenSig.connect(self.openPortBtn.statusSwitch)


    def updateAvailablePortSig(self,ports):
        self.INS_GPS_message.setStatusMessage('端口更新', 'orange', 'white')
        self.INS_port_select_.clear()
        for port in ports:
            self.INS_port_select_.addItem(port)

    def cmdSendWindow(self):
        sender = self.sender()
        if sender.text() == '发送':
            Str=self.INS_cmd_send_message.toPlainText()
            if Str == '':
                self.INS_GPS_message.setStatusMessage('请输入发送消息', 'orange', 'white')
            else:
                Str=Str.replace('\n','\r\n')
                flag=self.writePort(Str)
                if  flag is True:
                    self.INS_GPS_message.setStatusMessage('消息发送成功', 'green', 'white')
                elif flag is False:
                    self.INS_GPS_message.setStatusMessage('消息发送失败！', 'red', 'white')
        elif sender.text()=='清除':
            self.INS_cmd_send_message.clear()

    def saveConfig(self):
        flage=self.writePort(self.command['saveConfig'])
        if flage:
            self.INS_GPS_message.setStatusMessage('保存成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('保存失败', 'red', 'white')

    def comProcotolConfig(self):
        cmd='$cmd,set,'+self.INS_COM_protocol_config_COM_text.currentText()+','+self.INS_COM_protocol_config_baudrate_text.currentText()+','\
            +self.INS_COM_protocol_config_parity_text.currentText()+',8,'+self.INS_COM_protocol_config_stopbit_text.currentText()+','\
            +self.INS_COM_protocol_config_mode_text.currentText()+','+self.INS_COM_protocol_config_type_text.currentText()+'*ff'
        flage = self.writePort(cmd)
        if flage:
            self.INS_GPS_message.setStatusMessage('配置成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('配置失败', 'red', 'white')

    def comOutputConfig(self):
        cmd='$cmd,output,'+self.INS_COM_output_config_COM_text.currentText()+','+self.INS_COM_output_config_type_text.currentText()+','\
            +self.rate2speed[self.INS_COM_output_config_rate_text.currentText()]+'*ff'
        flage = self.writePort(cmd)
        if flage:
            self.INS_GPS_message.setStatusMessage('配置成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('配置失败', 'red', 'white')

    def comThroughConfig(self):
        cmd='$cmd,through,'+self.INS_COM_through_config_COM_text.currentText()+','+self.INS_COM_through_config_type_text.currentText()+','\
            +self.rate2speed[self.INS_COM_through_config_rate_text.currentText()]+'*ff'
        flage = self.writePort(cmd)
        if flage:
            self.INS_GPS_message.setStatusMessage('配置成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('配置失败', 'red', 'white')

    def inquirtConfig(self):
        sender=self.sender()
        flage=False
        if sender.text() == '导航状态':
            flage=self.writePort(self.command['navigationStatus'])
        elif sender.text() == 'COM配置':
            flage=self.writePort(self.command['COMConfigure'])
        elif sender.text() == 'COM输出':
            flage=self.writePort(self.command['COMOutput'])
        elif sender.text() == 'GNSS杆臂':
            flage=self.writePort(self.command['GNSSLeverarm'])
        elif sender.text() == '里程计状态':
            flage=self.writePort(self.command['odometerStatus'])
        if flage:
            self.INS_GPS_message.setStatusMessage('查询成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('查询失败', 'red', 'white')

    def modeConfig(self):
        sender=self.sender()
        flage=False
        if sender.text() == '千寻':
            flage=self.writePort(self.command['QX'])
        elif sender.text() == '基站':
            flage=self.writePort(self.command['radioStation'])
        if flage:
            self.INS_GPS_message.setStatusMessage('配置成功', 'green', 'white')
        elif flage is False:
            self.INS_GPS_message.setStatusMessage('配置失败', 'red', 'white')

    def switchPort(self):
        if self.myPort.portOpen == False:
            selectPortIndex=self.INS_port_select_.currentIndex()
            baudrate=self.INS_port_select_baudrate_.currentText()
            stopbits=self.INS_port_select_stopbit_.currentText()
            parity = self.INS_port_select_parity_.currentText()
            portConfig={'portIndex':selectPortIndex,'baudrate':baudrate,
                        'stopbits':stopbits,'parity':parity}
            if self.myPort.openPort(portConfig):
                self.myPort.portOpen = True
                self.INS_GPS_message.setStatusMessage('端口打开成功', 'green', 'white')
            else:
                self.INS_GPS_message.setStatusMessage('端口打开失败！','red','white')
                self.openPortBtn.statusSwitch(False)
        elif self.myPort.portOpen==True:
            self.myPort.closePort()
            self.INS_GPS_message.setStatusMessage('端口关闭成功', 'green', 'white')


    def closePort(self):
        self.openPortBtn.statusSwitch(False)

    def infoReceiveWindow(self,msg):
        self.INS_information_receive_message.append(msg)
        self.INS_information_receive_message.moveCursor(QTextCursor.End)

    def infoReceiveWindowClear(self):
        self.INS_information_receive_message.clear()

    def displayGPSData(self,data):
        self.INS_GPS_data_text.setText(data)

    def writePort(self,data):
        if self.myPort.portOpen:
            try:
                if type(data) is str:
                    self.myPort.port.write(bytes(data+'\n\r', encoding='utf-8'))
                elif type(data) is list:
                    for cmd in data:
                        self.myPort.port.write(bytes(cmd + '\n\r', encoding='utf-8'))
                return True
            except:
                return False

        else:
            self.INS_GPS_message.setStatusMessage('端口未打开！','red','white')
            return None


if __name__ == '__main__':
    app = QApplication(sys.argv)
    portInfo=QSerialPortInfo()
    myWin=MyWindows()
    dark_stylesheet = qdarkstyle.load_stylesheet_pyqt5()
    app.setStyleSheet(dark_stylesheet)
    myWin.show()

    app.exec_()
    sys.exit(0)