import math
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from threading import Thread

class MPU(Thread):
	def __init__(self):
		super().__init__()
		self.sumGyroZ=0
		self.mpuComVal=0.0004
		self.mpu = MPU9250(
			address_ak=AK8963_ADDRESS,
			address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
			address_mpu_slave=None,
			bus=1,
			gfs=GFS_1000,
			afs=AFS_8G,
			mfs=AK8963_BIT_16,
			mode=AK8963_MODE_C100HZ)

		self.mpu.configure() # Apply the settings to the registers.
		# 		#enter accBiases:
		#print("accBiases", self.calibrateAccOffset(2000))
		self.accBiases=[0.4439825439453125, 0.352824462890625, -1.9188690185546875]
		#gyroCalibration
		calibrationData=self.calibrateGyro(2000)
		self.gyroBiases=calibrationData[0:3]
		self.sumGyroX, self.sumGyroY=calibrationData[3]
		#print(self.sumGyroX,self.sumGyroY)
		#set Mpu Frequency
		self.freq=100

	def setMPUComVal(self, val):
		self.mpuComVal=val

	def calibrateGyro(self, cycles):
		sumX=0
		sumY=0
		sumZ=0
		sumXAngle=0
		sumYAngle=0

		i=0
		while i<=cycles:
			data=self.mpu.readGyroscopeMaster()
			sumX+=data[0]
			sumY+=data[1]
			sumZ+=data[2]

			accData=self.mpu.readAccelerometerMaster()
			accX=accData[0]*-1-self.accBiases[0]
			accY=accData[1]*-1-self.accBiases[1]
			accZ=accData[2]*-1-self.accBiases[2]
			totalVector=math.sqrt(accX**2+accY**2+accZ**2)
			angleAccX=math.asin(accY/totalVector)*180/math.pi*-1
			angleAccY=math.asin(accX/totalVector)*180/math.pi
			sumXAngle+=angleAccX
			sumYAngle+=angleAccY
			i+=1
			time.sleep(.01)
		return [sumX/cycles, sumY/cycles, sumZ/cycles, (sumXAngle/cycles, sumYAngle/cycles)]

	def resetGyro(self):
		calibrationData = self.calibrateGyro(2000)
		self.gyroBiases = calibrationData[0:3]
		self.sumGyroX, self.sumGyroY = calibrationData[3]
		self.sumGyroZ=0

	def calibrateAccOffset(self, cycles):
		sumX=0
		sumY=0
		sumZ=0
		i=0
		while i<=cycles:
			data=self.mpu.readAccelerometerMaster()
			sumX+=data[0]
			sumY+=data[1]
			sumZ+=data[2]
			i+=1
		return [-1*sumX/cycles,-1*sumY/cycles,-1*sumZ/cycles-1]

	def run(self):
		n=0
		self.sumGyroZ=0
		while True:
			lastTime=time.time()

			#gyro
			gyroData=self.mpu.readGyroscopeMaster()
			gyroX=gyroData[0]-self.gyroBiases[0]
			gyroY=gyroData[1]-self.gyroBiases[1]
			gyroZ=gyroData[2]-self.gyroBiases[2]

			#acc
			accData=self.mpu.readAccelerometerMaster()
			accX=accData[0]*-1-self.accBiases[0]
			accY=accData[1]*-1-self.accBiases[1]
			accZ=accData[2]*-1-self.accBiases[2]
			totalVector=math.sqrt(accX**2+accY**2+accZ**2)
			angleAccX=math.asin(accY/totalVector)*180/math.pi*-1
			angleAccY=math.asin(accX/totalVector)*180/math.pi

			#gyro+acc
			gyroX=gyroX*(1-self.mpuComVal)+angleAccX*self.mpuComVal
			gyroY=gyroY*(1-self.mpuComVal)-angleAccY*self.mpuComVal

			#integral
			self.sumGyroX+=gyroX/self.freq
			self.sumGyroY+=gyroY/self.freq
			self.sumGyroZ+=gyroZ/self.freq

			self.sumGyroX+=self.sumGyroY*math.sin(gyroZ/self.freq*(math.pi/180))
			self.sumGyroY-=self.sumGyroX*math.sin(gyroZ/self.freq*(math.pi/180))

			n+=1
			while time.time()<lastTime+1/self.freq:
				pass

			if n==10 and __name__=="__main__":
				n=0
				#print(accX, accY, accZ, totalVector)
				print(angleAccX, angleAccY)
				#print(self.sumGyroX, self.sumGyroY, self.sumGyroZ)

if __name__=="__main__":
	m=MPU()
	#print(m.calibrateAccOffset(2000))
	m.start()
