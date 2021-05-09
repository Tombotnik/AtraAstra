import pickle
import select
import socket
import struct
import sys
import threading
import time

import adafruit_ads1x15.ads1115 as ADS
import adafruit_pca9685
import board
import busio
import cv2
import matplotlib.pyplot as plt
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit
from picamera import PiCamera
from picamera.array import PiRGBArray
from simple_pid import PID

from mpu import MPU

socket.setdefaulttimeout(2)
plot=True

class Server:
    def __init__(self):
        self.previousPowerLevel=0
        self.powerLevel=0

        self.xAngleDeflection = 5
        self.yAngleDeflection = 5

        self.mpu=MPU()
        self.mpu.start()

        self.pidSampletime=0.01
        self.xPID=PID(0.01*72,0.01*19,0.01*9,0)
        self.xPID.sample_time=self.pidSampletime
        self.xPID.output_limits = (-20, 20)
        self.yPID=PID(0.01*81,0.01*13,0.01*14,0)
        self.yPID.sample_time=self.pidSampletime
        self.yPID.output_limits = (-20, 20)
        self.xPID.auto_mode=False
        self.yPID.auto_mode=False

        self.ended=False
        self.s=socket.socket()
        self.s.bind(("",5005))
        self.s.listen()

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = 2000
        self.kit = ServoKit(channels=8)

        self.m1=self.kit.servo[0]
        self.m2=self.kit.servo[1]
        self.m3=self.kit.servo[2]
        self.m4=self.kit.servo[3]
        self.m1Val=0
        self.m2Val=0
        self.m3Val=0
        self.m4Val=0

        self.arm()
        threading.Thread(target=self.pidCorrectionThread).start()

        self.activeClient=False
        self.awaitClients()

    def measureVoltageThread(self, con):
        self.ads = ADS.ADS1115(self.i2c)
        while not self.ended and self.connected:
            time.sleep(5)
            self.voltageChan = AnalogIn(self.ads, ADS.P1)
            voltage=self.voltageChan.voltage * (683 + 220) / 220
            value=int((voltage-4*3.7)*50)
            try:
                con.send(("Voltage: "+str(value)).encode())
            except:
                pass

    def awaitClients(self):
        while not self.ended:
            try:
                (con,adr)=self.s.accept()
                (videoCon,_)=self.s.accept()
                print("Connection by "+str(adr))
                threading.Thread(target=self.authorizeClient,args=[adr,con,videoCon]).start()
            except:
                pass

    def authorizeClient(self,adr,con,videoCon):
        #con.setblocking(0)
        ready = select.select([con], [], [], 4)
        if ready[0]:
            data = con.recv(1024).decode()
            if data=="hello" and not self.activeClient:
                self.adr=adr
                self.con=con
                self.activeClient=True
                self.connected=True
                threading.Thread(target=self.measureVoltageThread, args=[con]).start()
                videoResponse = select.select([videoCon], [], [], 4)
                if videoResponse[0]:
                    data=videoCon.recv(1024).decode()
                    if data=="video":
                        threading.Thread(target=self.sendImages, args=[videoCon]).start()
                self.handleClient()
        con.close()
        print("Connection closed "+str(adr)+"\n")


    def sendImages(self, videoCon):
        if self.connected:
            with PiCamera() as camera:
                camera.resolution = (640, 480)
                camera.framerate = 30
                rawCapture = PiRGBArray(camera, size=(640,480))
                for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                    if not self.connected:
                        break
                    img = frame.array
                    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img=cv2.flip(img, 0)
                    data = pickle.dumps(img)
                    size = len(data)
                    videoCon.sendall(struct.pack("Q", size) + data)
                    rawCapture.truncate(0)
                    time.sleep(1/30)


    def end(self):
        print("ending Server")
        self.connected=False
        try:
            self.xPID.auto_mode=False
            self.yPID.auto_mode=False
            self.con.send("close: Server trennt Verbindung".encode())
        except:
            pass
        self.con.close()
        self.activeClient=False
        self.ended=True
        sys.exit()

    def handleClient(self):
        try:
            while (not self.ended):
                ready = select.select([self.con], [], [], 4)
                if ready[0]:
                    mes=self.con.recv(1024).decode()
                    mes=mes.split(" . ")[-2]
                    if(mes):
                        #print(mes)
                        mesSplit=mes.split(" ")
                        if(mes=="close"):
                            raise
                        elif(mes=="kill"):
                            self.powerLevel=0
                            self.stopMotors()
                            print("Stop everything")
                        elif (mes == "resetGyro"):
                            self.mpu.resetGyro()
                        elif(mes=="end"):
                            self.end()
                        elif(mesSplit[0]=="setAllCalc"):
                            self.setAllMotorsCalc(int(mesSplit[1]))
                            print("Calc: Setze alle Motoren auf "+mesSplit[1])
                        elif(mesSplit[0]=="setAllRaw"):
                            self.setAllMotorsRaw(int(mesSplit[1]))
                            print("Raw: Setze alle Motoren auf "+mesSplit[1])
                        elif(mesSplit[0]=="setMotorCalc"):
                            if(mesSplit[1]=="M1" or mesSplit[1]=="M2" or mesSplit[1]=="M3" or mesSplit[1]=="M4"):
                                self.setMotorRaw(mesSplit[1],int(mesSplit[2])+80)
                                print("Motor " + mesSplit[1] + " DCalc:"+mesSplit[2])
                        elif(mesSplit[0]=="setMotorRaw"):
                            if(mesSplit[1]=="M1" or mesSplit[1]=="M2" or mesSplit[1]=="M3" or mesSplit[1]=="M4"):
                                self.setMotorRaw(mesSplit[1],int(mesSplit[2]))
                                print("Motor " + mesSplit[1] + " DRaw:"+mesSplit[2])
                        elif(mesSplit[0]=="pid"):
                            if mesSplit[1]=="X":
                                pid = self.xPID
                                pidCommand = mes.split("X")[1][1:]
                            else:
                                pid=self.yPID
                                pidCommand = mes.split("Y")[1][1:]
                            print(pidCommand)
                            if pidCommand.startswith("p:"):
                                print(pidCommand.split("p:")[1])
                                pid.Kp=float(pidCommand.split("p:")[1])
                            elif pidCommand.startswith("i:"):
                                print(pidCommand.split("i:")[1])
                                pid.Ki=float(pidCommand.split("i:")[1])
                            elif pidCommand.startswith("d:"):
                                print(pidCommand.split("d:")[1])
                                pid.Kd=float(pidCommand.split("d:")[1])
                            print("set pid Axis:"+ mesSplit[1]+" K:"+ str(pid.Kp) + " I:"+ str(pid.Ki) + " D:" + str(pid.Kd))
                        elif (mesSplit[0] == "MPUSlider"):
                            print(mesSplit[1])
                            self.mpu.setMPUComVal(float(mesSplit[1]))
                        elif(mesSplit[0]=="keyCom"):
                            print(mes)
                            if(mes.split("p:")[1]):
                                self.powerLevel=int(mes.split("p:")[1])/2
                            x=mes.split("X:")[1].split(" ")[0]
                            if (x=="f"):
                                self.xPID.setpoint = self.xAngleDeflection
                                print(self.xPID.components)
                            elif (x=="b"):
                                self.xPID.setpoint = -1*self.xAngleDeflection
                            else:
                                self.xPID.setpoint = 0
                            y = mes.split("Y:")[1].split(" ")[0]
                            if (y=="l"):
                                self.yPID.setpoint = self.yAngleDeflection
                            elif (y=="r"):
                                self.yPID.setpoint = -1*self.yAngleDeflection
                            else:
                                self.yPID.setpoint = 0

        except Exception as e:
            print("error "+str(e))
            self.setAllMotorsCalc(0)
            self.connected=False
            try:
                self.con.send("close: Server trennt Verbindung".encode())
            except:
                pass
            self.con.close()
            self.activeClient=False
            print("Connection closed "+str(self.adr))

    def pidCorrectionThread(self):
        global plot
        print(plot)
        if plot:
            plotListX=[]
            plotListY=[]
            cycles=[]
            i=0
        while not self.ended:
            self.xAngle=self.mpu.sumGyroX
            self.yAngle=self.mpu.sumGyroY
            self.zAngle=self.mpu.sumGyroZ
            try:
                self.con.send(("Gyro: " + str(self.xAngle) + " " + str(self.yAngle) + " " + str(self.zAngle)).encode())
            except:
                pass
            i+=1
            if(self.powerLevel>5):
                self.xPID.auto_mode=True
                self.yPID.auto_mode=True
                xPIDOutput=self.xPID(self.xAngle)
                #xPIDOutput=0
                yPIDOutput=self.yPID(self.yAngle)
                #yPIDOutput = 0
                #print(xPIDOutput)
            else:
                self.xPID.auto_mode=False
                self.yPID.auto_mode=False
                xPIDOutput=0
                yPIDOutput=0
            cycles+=[i]
            if self.xPID.auto_mode:
                self.setMotorCalc("M1",self.powerLevel-xPIDOutput-yPIDOutput)
                self.setMotorCalc("M2",self.powerLevel-xPIDOutput+yPIDOutput)
                self.setMotorCalc("M3",self.powerLevel+xPIDOutput+yPIDOutput)
                self.setMotorCalc("M4",self.powerLevel+xPIDOutput-yPIDOutput)
            else:
                self.setAllMotorsCalc(self.powerLevel)
            if plot:
                plotListX+=[self.xAngle]
                plotListY+=[self.yAngle]
                #print(xPIDOutput,yPIDOutput)
            time.sleep(.01)
        self.stopMotors()
        if(plot):
            plt.plot(cycles,plotListX, label="X-Values")
            plt.plot(cycles,plotListY, label="Y-Values")
            plt.xlabel("Cycles")
            plt.ylabel("Degrees")
            plt.title("Imu during flight")
            plt.legend()
            plt.savefig("Imu.png")
            #print(plotListX)

    def arm(self):
        self.setAllMotorsRaw(80)
        time.sleep(2)

    def stopMotors(self):
        self.setAllMotorsRaw(80)

    def setAllMotorsCalc(self,n):
        self.setAllMotorsRaw(80+n)

    def setAllMotorsRaw(self,n):
        self.m1.angle=n
        self.m2.angle=n
        self.m3.angle=n
        self.m4.angle=n

    def setMotorRaw(self,m,n):
        if(m=="M1"):
            self.m1.angle=n
        elif(m=="M2"):
            self.m2.angle=n
        elif(m=="M3"):
            self.m3.angle=n
        elif(m=="M4"):
            self.m4.angle=n

    def setMotorCalc(self,m,n):
        self.setMotorRaw(m,80+n)

s=Server()
