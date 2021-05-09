import time
from threading import Thread

import smbus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman


class KalmanMPU(Thread):
	def __init__(self, debug=False):
		super(KalmanMPU, self).__init__()
		self.address = 0x68
		self.debug=debug
		self.ended=False
		self.bus = smbus.SMBus(1)
		self.imu = MPU9250.MPU9250(self.bus, self.address)
		self.imu.setLowPassFilterFrequency("AccelLowPassFilter184")
		self.imu.begin()
		self.imu.loadCalibDataFromFile("/home/pi/calib_real4.json")

		self.sensorfusion = kalman.Kalman()

		self.imu.readSensor()
		self.imu.computeOrientation()
		self.sensorfusion.roll = self.imu.roll
		self.sensorfusion.pitch = self.imu.pitch
		self.sensorfusion.yaw = self.imu.yaw

	def end(self):
		self.ended=True

	def run(self):
		currTime = time.time()
		while not self.ended:
			pitchVal=0
			rollVal=0
			yawVal=0
			cycles=10
			for i in range(0,cycles):
				self.imu.readSensor()
				self.imu.computeOrientation()
				newTime = time.time()
				dt = newTime - currTime
				currTime = newTime
				self.sensorfusion.computeAndUpdateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], -1*self.imu.AccelVals[2], self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],\
														self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt)
				pitchVal+=self.sensorfusion.pitch
				rollVal+=self.sensorfusion.roll*-1
				yawVal+=self.sensorfusion.yaw

			self.pitch=pitchVal/cycles
			self.roll=rollVal/cycles
			self.yaw=yawVal/cycles
			if self.debug:
				print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw))

if __name__=="__main__":
	mpu=KalmanMPU()
	mpu.start()