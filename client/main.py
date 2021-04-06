import pickle
import struct
import cv2
from PyQt5.QtGui import QImage, QPixmap

from easySocketModule import NetworkScanner, EasyTCPClient
import socket
import threading
import time

class mainProgram:
    def __init__(self):
        self.connected=False
        self.__listeners=[]
        self.__powerBarListeners=[]
        self.__gyroListeners=[]
        self.__cameraImageListeners=[]
        self.netScanner=NetworkScanner()
        self.forwardState=False
        self.leftState=False
        self.backwardsState=False
        self.rightState=False
        self.upState=False
        self.downState=False
        self.turnLeftState=False
        self.turnRightState=False
        self.lastX=""
        self.lastY=""
        self.lastZ=""
        self.lastTurn=""
        self.active=True
        self.foundIPs=[]
        self.powerSliderValue=""

    def __callListeners(self):
        for f in self.__listeners:
            f()

    def __callPowerBarListeners(self):
        for f in self.__powerBarListeners:
            f(int(self.voltage))

    def __callGyroListeners(self):
        for f in self.__gyroListeners:
            f(self.gyroValues)
            
    def addListener(self, listener):
        self.__listeners+=[listener]

    def addGyroListener(self, listener):
        self.__gyroListeners+=[listener]

    def addPowerBarListener(self, listener):
        self.__powerBarListeners+=[listener]

    def addCameraImageListener(self, listener):
        self.__cameraImageListeners+=[listener]

    def setConnectButtonListener(self, listener):
        try:
            self.connectButtonListener=listener
        except:
            pass

    def callConnectButtonListener(self):
        self.connectButtonListener()

    def processCommand(self, command):
        if self.connected:
            self.evaluateCommand(command)

    def updateCameraListeners(self, img):
        for listener in self.__cameraImageListeners:
            listener(img)

    def netScan(self):
        self.foundIPs=self.netScanner.scan()
        self.__callListeners()

    def connect(self, ip="10.42.0.226"):
        if ip=="":
            ip="10.42.0.226"
        try:
            if not (self.connected):
                self.connection=socket.socket()
                self.videoConnection=socket.socket()
                self.connection.settimeout(5)
                self.videoConnection.settimeout(5)
                self.connection.connect((ip,5005))
                self.videoConnection.connect((ip,5005))
                self.connected=True
                self.connection.send("hello".encode())
                time.sleep(.3)
                self.videoConnection.send("video".encode())
                self.callConnectButtonListener()

                threading.Thread(target=self.receiveMessages).start()
                threading.Thread(target=self.receiveImages).start()
        except:
            pass
        self.__callListeners()

    def disconnect(self):
        try:
            self.connection.send("close".encode())
            self.connection.close()
        except:
            pass
        self.connected=False
        self.callConnectButtonListener()

    def sendMessage(self, message):
        try:
            self.connection.send((message+" . ").encode())
            if(message=="end"):
                self.disconnect()
        except:
            self.connected=False

    def receiveMessages(self):
        while self.connected:
            try:
                message=self.connection.recv(1024).decode()
                if message.startswith("Gyro:"):
                    self.gyroValues=message[6:].split(" ")
                    self.gyroValues=[float(e) for e in self.gyroValues]
                    self.__callGyroListeners()
                elif (message.startswith("Voltage:")):
                    self.voltage=float(message[9:])
                    self.__callPowerBarListeners()
                else:
                    pass
                    #print("Unknown message: "+message)
                    #print(len(message))
            except Exception as e:
                pass

    def receiveImages(self):
        payload_size = struct.calcsize("Q")
        data = b""
        print("payload_size: {}".format(payload_size))
        while True:
            if self.connected:
                try:
                    while len(data) < payload_size:
                        if not self.connected: return
                        data += self.videoConnection.recv(4*1024)
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack("Q", packed_msg_size)[0]
                    while len(data) < msg_size:
                        if not self.connected: return
                        data += self.videoConnection.recv(4*1024)
                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
                    height, width, channel = frame.shape
                    bytesPerLine = 3 * width
                    qImg = QImage(frame.data, width, height, bytesPerLine, QImage.Format_RGB888)
                    qImg = QPixmap(qImg)
                    self.updateCameraListeners(qImg)
                    #frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
                    #cv2.imshow('ImageWindow', frame)
                    #cv2.waitKey(1)
                except Exception as e:
                    print(e)

    def evaluateCommand(self, command):
        commandSplit=command.split(" ")
        if(command=="kill" or command=="safeLanding"):
            self.sendMessage(command)
            return
        elif(command=="closeSocket"):
            self.connection.close()
        elif(commandSplit[0]=="PowerSlider"):
            self.powerSliderValue=commandSplit[1]
        elif (commandSplit[0] == "PIDSlider"):
            self.sendMessage(command[10:])
        elif (commandSplit[0] == "MPUSlider"):
            self.sendMessage(command)
        elif(command=="f1"):
            self.forwardState=True
            self.lastX="f"
        elif(command=="b1"):
            self.backwardsState=True
            self.lastX="b"
        elif(command=="l1"):
            self.leftState=True
            self.lastY="l"
        elif(command=="r1"):
            self.rightState=True
            self.lastY="r"
        elif(command=="u1"):
            self.upState=True
            self.lastZ="u"
        elif(command=="d1"):
            self.downState=True
            self.lastZ="d"
        elif(command=="tl1"):
            self.turnLeftState=True
            self.lastTurn="l"
        elif(command=="tr1"):
            self.turnRightState=True
            self.lastTurn="r"

        elif(command=="f0"):
            self.forwardState=False
        elif(command=="b0"):
            self.backwardsState=False
        elif(command=="l0"):
            self.leftState=False
        elif(command=="r0"):
            self.rightState=False
        elif(command=="d0"):
            self.downState=False
        elif(command=="u0"):
            self.upState=False
        elif(command=="tl0"):
            self.turnLeftState=False
        elif(command=="tr0"):
            self.turnRightState=False
        self.generateMessage()

    def generateMessage(self):
        mess=""
        if(self.active==True):
                #x
                if(self.forwardState==True or self.backwardsState==True):
                    if(self.lastX=="f"):
                        if(self.forwardState==True):
                            mess+="X:f "
                        else:
                            mess+="X:b "
                    else:
                        if(self.backwardsState==True):
                            mess+="X:b "
                        else:
                            mess+="X:f "
                else:
                    mess+="X:- "

                #y
                if(self.leftState==True or self.rightState==True):
                    if(self.lastY=="l"):
                        if(self.leftState==True):
                            mess+="Y:l "
                        else:
                            mess+="Y:r "
                    else:
                        if(self.rightState==True):
                            mess+="Y:r "
                        else:
                            mess+="Y:l "
                else:
                    mess+="Y:- "

                #z
                if(self.upState==True or self.downState==True):
                    if(self.lastZ=="u"):
                        if(self.upState==True):
                            mess+="Z:u "
                        else:
                            mess+="Z:d "
                    else:
                        if(self.downState==True):
                            mess+="Z:d "
                        else:
                            mess+="Z:u "
                else:
                    mess+="Z:- "
                #t
                if(self.turnLeftState==True or self.turnRightState==True):
                    if(self.lastTurn=="l"):
                        if(self.turnLeftState==True):
                            mess+="T:l "
                        else:
                            mess+="T:r "
                    else:
                        if(self.turnRightState==True):
                            mess+="T:r "
                        else:
                            mess+="T:l "
                else:
                    mess+="T:- "
                mess+="p:"+self.powerSliderValue

                #send
                self.sendMessage("keyCom "+mess)
                mess=""

    def getConnectionState(self,text):
        if(text=="Connect"):
            return "Disconnect"
        else:
            return "Connect"
