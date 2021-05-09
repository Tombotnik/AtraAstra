import json
import os
import time
import MPU6050
import struct
import smbus
import math
from threading import Thread

class MPU(Thread):
        def __init__(self, debug=False):
                super().__init__()
                self.accBiases=[-86,9974,327]
                #(-261.5295, -40.716, -17.288)
                self.debug=debug
                self.freq=100
                self.bus = smbus.SMBus(1)
                self.address = 0x68
                self.firstAngles=True

                self.bus.write_byte_data(self.address, 0x6b, 0)
                self.bus.write_byte_data(self.address, 0x00, 0)

                self.bus.write_byte_data(self.address, 0x1C, 0b00001000)
                self.bus.write_byte_data(self.address, 0x10, 0)

                self.bus.write_byte_data(self.address, 0x1B, 0)
                self.bus.write_byte_data(self.address, 0x08, 0)

                self.gyroOffsets=self.calibrateGyro()

                self.mpu = MPU6050.MPU6050()
                self.mpu.setGResolution(2)
                self.bus.write_byte_data(self.address, 0x1C, 0b00001000)
                self.bus.write_byte_data(self.address, 0x10, 0)
                self.mpu.setSampleRate(self.freq)
                time.sleep(0.01)

        def calibrateAccelerometer(self, cycles=2000):
                i = 0
                accX = 0
                accY = 0
                accZ = 0
                while (i < cycles):
                        rawAccX, rawAccY, rawAccZ = self.getRawAccData()
                        accX+=rawAccX
                        accY+=rawAccY
                        accZ+=rawAccZ-8192
                        i += 1
                self.accBiases=[accX/cycles, accY/cycles, accZ/cycles]
                print("calibrated Accelerometer")
                return accX/cycles, accY/cycles, accZ/cycles

        def measureAccelerometerAndSave(self):
                self.calibrateAccelerometer()
                with open("/calibrate.json", "w+") as f:
                        json.dump(self.accBiases)

        def readAccelerometerBiasesFromFile(self):
                with open("/calibrate.json", "r") as f:
                        self.accBiases=json.load(f)
                        print(self.accBiases)

        def calibrateGyro(self, cycles=2000):
                i = 0
                gyroX = 0
                gyroY = 0
                gyroZ = 0
                while (i < cycles):
                        rawGyroX, rawGyroY, rawGyroZ=self.getRawGyroData()
                        gyroX += rawGyroX
                        gyroY += rawGyroY
                        gyroZ += rawGyroZ
                        i += 1
                rawXOffset = gyroX / cycles
                rawYOffset = gyroY / cycles
                rawZOffset = gyroZ / cycles
                self.gyroOffsets=[rawXOffset, rawYOffset, rawZOffset]
                print("calibrated Gyroscope")
                return rawXOffset, rawYOffset, rawZOffset

        def read_byte(self, reg):
                return self.bus.read_byte_data(self.address, reg)

        def read_word(self, reg):
                h = self.bus.read_byte_data(self.address, reg)
                l = self.bus.read_byte_data(self.address, reg + 1)
                value = (h << 8) + l
                return value

        def read_word_2c(self, reg):
                val = self.read_word(reg)
                if (val >= 0x8000):
                        return -((65535 - val) + 1)
                else:
                        return val

        def getRawGyroData(self):
                rawGyroX = self.read_word_2c(0x43)
                rawGyroY = self.read_word_2c(0x45)
                rawGyroZ = self.read_word_2c(0x47)
                return rawGyroX, rawGyroY, rawGyroZ

        def getRawAccData(self):
                rawAccX = self.read_word_2c(0x3b)-self.accBiases[0]
                rawAccY = self.read_word_2c(0x3d)-self.accBiases[1]
                rawAccZ = self.read_word_2c(0x3f)-self.accBiases[2]
                return rawAccX, rawAccY, rawAccZ

        def run(self):
                self.mpu.enableFifo(True, 0b01110000)
                pitchRawSumValue = 0
                rollRawSumValue = 0
                yawRawSumValue = 0
                self.yawDegreeSumValue=0
                time1=time.time()
                while 1:
                        pitchRawValue=0
                        rollRawValue=0
                        yawRawValue=0
                        fifoCount = self.mpu.readFifoCount()
                        if fifoCount > 23:
                                if fifoCount == 1024:
                                        print("Overflow")
                                        raise ("overflow Error")
                                data = self.mpu.readDataFromFifo(24)
                                if data:
                                        short = struct.unpack(">" + "h" * 12, memoryview(bytearray(data)))
                                        for x in range(0, len(short), 3):
                                                pitchRawValue+=short[x]
                                        for y in range(1, len(short), 3):
                                                rollRawValue+=short[y]
                                        for z in range(2, len(short), 3):
                                                yawRawValue+=short[z]
                        timeDelay=time.time()-time1
                        time1=time.time()

                        pitchRawSumValue+=pitchRawValue
                        rollRawSumValue+=rollRawValue
                        yawRawSumValue+=yawRawValue
                        pitchRawSumValue-=timeDelay*self.freq/2*self.gyroOffsets[0]
                        rollRawSumValue-=timeDelay*self.freq/2*self.gyroOffsets[1]
                        yawRawSumValue-=timeDelay*self.freq/2*self.gyroOffsets[2]
                        pitchDegreeSumValue=pitchRawSumValue/65.5/self.freq
                        rollDegreeSumValue=rollRawSumValue/65.5/self.freq
                        self.yawDegreeSumValue=yawRawSumValue/65.5/self.freq
                        pitchDegreeSumValue += rollDegreeSumValue * math.sin(yawRawSumValue/65.5 * (math.pi / 180))
                        rollDegreeSumValue -= pitchDegreeSumValue * math.sin(yawRawSumValue/65.5 * (math.pi / 180))

                        rawAccX, rawAccY, rawAccZ=self.getRawAccData()
                        acc_total_vector = math.sqrt((rawAccX ** 2) + (rawAccY ** 2) + (rawAccZ ** 2))
                        angle_pitch_acc = math.asin(rawAccY / acc_total_vector) / math.pi * 180
                        angle_roll_acc = math.asin(rawAccX / acc_total_vector) / (-math.pi) * 180

                        if not self.firstAngles:
                                self.anglePitchOutput = pitchDegreeSumValue * 0.9996 + angle_pitch_acc * 0.0004
                                self.angleRollOutput = rollDegreeSumValue * 0.9996 + angle_roll_acc * 0.0004
                        else:
                                self.anglePitchOutput = angle_pitch_acc
                                self.angleRollOutput = angle_roll_acc
                                self.firstAngles = False
                        if self.debug==True:
                                print(self.anglePitchOutput, self.angleRollOutput, self.yawDegreeSumValue)
                                #print(angle_pitch_acc, angle_roll_acc)
                                #print(rawAccX, rawAccZ)
                                #print(self.yawDegreeSumValue)
                        self.pitch=self.anglePitchOutput
                        self.roll=self.angleRollOutput
                        self.yaw=self.yawDegreeSumValue


if __name__=="__main__":
        mpu=MPU(debug=True)
        #print(mpu.calibrateAccelerometer())
        mpu.start()
