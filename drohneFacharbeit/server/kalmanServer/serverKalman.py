import pickle
import socket
import struct

import cv2
import numpy as np
import time
import threading
import select
from flask import Flask, render_template, Response
import sys

import matplotlib.pyplot as plt
from simple_pid import PID
from kalmanMPU9250Lib import KalmanMPU

import board
import busio

import adafruit_pca9685
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit
from picamera.array import PiRGBArray
from picamera import PiCamera
import io

socket.setdefaulttimeout(2)
plot=True

#pitchPID P:0.01*93 I:0.01*21 D:0.01*16 Com:0.000001*40
#roll PID P:0.01*32 I:0.01*67 D:0.01*7 Com:0.00001*40

class Server:
    def __init__(self):
        self.previousPowerLevel=0
        self.powerLevel=0

        self.rollAngleDeflection = 5
        self.pitchAngleDeflection = 5

        self.mpu=KalmanMPU()
        self.mpu.start()

        self.pidSampletime=0.01
        self.rollPID=PID(0.01*32,0.01*67,0.01*7,0)
        self.rollPID.sample_time=self.pidSampletime
        self.rollPID.output_limits = (-20, 20)
        self.pitchPID=PID(0.01*93,0.01*21,0.01*16,0)
        self.pitchPID.sample_time=self.pidSampletime
        self.pitchPID.output_limits = (-20, 20)
        self.rollPID.auto_mode=False
        self.pitchPID.auto_mode=False

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
                        self.videoCon=videoCon
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
            self.rollPID.auto_mode=False
            self.pitchPID.auto_mode=False
            self.mpu.end()
            self.con.send("close: Server trennt Verbindung".encode())
        except:
            pass
        self.videoCon.close()
        self.con.close()
        self.s.close()
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
                            self.mpu.calibrateGyro()
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
                                pid = self.rollPID
                                pidCommand = mes.split("X")[1][1:]
                            else:
                                pid=self.pitchPID
                                pidCommand = mes.split("Y")[1][1:]
                            #print(pidCommand)
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
                            if (x=="l"):
                                self.rollPID.setpoint = self.xAngleDeflection
                                print(self.rollPID.components)
                            elif (x=="r"):
                                self.rollPID.setpoint = -1*self.xAngleDeflection
                            else:
                                self.rollPID.setpoint = 0
                            y = mes.split("Y:")[1].split(" ")[0]
                            if (y=="f"):
                                self.pitchID.setpoint = self.yAngleDeflection
                            elif (y=="b"):
                                self.pitchPID.setpoint = -1*self.yAngleDeflection
                            else:
                                self.pitchPID.setpoint = 0

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
            plotListRoll=[]
            plotListPitch=[]
            cycles=[]
            i=0
        while not self.ended:
            try:
                self.con.send(("Gyro: " + str(self.mpu.roll) + " " + str(self.mpu.pitch) + " " + str(self.mpu.yaw)).encode())
            except:
                pass
            i+=1
            if(self.powerLevel>5):
                self.rollPID.auto_mode=True
                self.pitchPID.auto_mode=True
                #rollPIDOutput=self.rollPID(self.mpu.roll)
                rollPIDOutput=0
                pitchPIDOutput=self.pitchPID(self.mpu.pitch)
                #pitchPIDOutput = 0
                #print(pitchPIDOutput)
            else:
                self.pitchPID.auto_mode=False
                self.rollPID.auto_mode=False
                rollPIDOutput=0
                pitchPIDOutput=0
            cycles+=[i]
            if self.rollPID.auto_mode:
                self.setMotorCalc("M1",self.powerLevel-rollPIDOutput+pitchPIDOutput)
                self.setMotorCalc("M2",self.powerLevel+rollPIDOutput+pitchPIDOutput)
                self.setMotorCalc("M3",self.powerLevel+rollPIDOutput-pitchPIDOutput)
                self.setMotorCalc("M4",self.powerLevel-rollPIDOutput-pitchPIDOutput)
            else:
                self.setAllMotorsCalc(self.powerLevel)
            if plot:
                plotListRoll+=[self.mpu.roll]
                plotListPitch+=[self.mpu.pitch]
            time.sleep(.01)
        self.stopMotors()
        if(plot):
            plt.plot(cycles,plotListRoll, label="Roll-Values")
            plt.plot(cycles,plotListPitch, label="Pictch-Values")
            plt.xlabel("Cycles")
            plt.ylabel("Degrees")
            plt.title("Imu during flight")
            plt.legend()
            plt.savefig("Imu.png")
            #print(plotListRoll)

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
