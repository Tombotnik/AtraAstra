from PyQt5 import QtWidgets, uic, QtCore, Qt, QtGui
import sys
from main import mainProgram
from pynput import keyboard
from PyQt5.QtCore import Qt, pyqtSignal
import math

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('droneGui.ui', self)
        self.mainProgram=mainProgram()
        self.mainProgram.addListener(self.updateGui)
        self.mainProgram.addGyroListener(self.updateGyro)
        self.mainProgram.addPowerBarListener(self.updatePowerBar)
        self.mainProgram.addCameraImageListener(self.updateCameraImage)
        self.mainProgram.setConnectButtonListener(self.updateConnectButton)
        self.batteryLabel.setAlignment(QtCore.Qt.AlignCenter)

        self.listener=keyboard.Listener(on_release = self.on_key_release, on_press=self.on_key_pressed)
        self.listener.start()

        self.addressComboBox = self.findChild(QtWidgets.QComboBox, 'addressComboBox')
        self.testingCheckBox = self.findChild(QtWidgets.QCheckBox, 'testingCheckBox')

        self.powerSlider = self.findChild(QtWidgets.QSlider, 'powerSlider')
        self.powerSlider.valueChanged.connect(self.powerSliderChanged)

        self.mpuComSlider = self.findChild(QtWidgets.QSlider, 'mpuComSlider')
        self.mpuComSlider.valueChanged.connect(self.mpuSliderChanged)

        self.pSlider = self.findChild(QtWidgets.QSlider, 'pSlider')
        self.pSlider.valueChanged.connect(lambda: self.pidSliderChanged(self.pLabel, self.pSlider, self.pOffsetEntry, self.pMultiplierEntry, "p"))

        self.iSlider = self.findChild(QtWidgets.QSlider, 'iSlider')
        self.iSlider.valueChanged.connect(lambda :self.pidSliderChanged(self.iLabel, self.iSlider, self.iOffsetEntry, self.iMultiplierEntry, "i"))

        self.dSlider = self.findChild(QtWidgets.QSlider, 'dSlider')
        self.dSlider.valueChanged.connect(lambda :self.pidSliderChanged(self.dLabel, self.dSlider, self.dOffsetEntry, self.dMultiplierEntry, "d"))

        self.manualCommand=self.findChild(QtWidgets.QLineEdit, 'manualCommand')
        #buttons
        self.netSearchButton = self.findChild(QtWidgets.QPushButton, 'netSearchButton')
        self.netSearchButton.pressed.connect(self.searchForAddresses)

        self.killButton = self.findChild(QtWidgets.QPushButton, 'killButton')
        self.killButton.pressed.connect(lambda:self.buttonPressed("kill"))

        self.resetGyroButton = self.findChild(QtWidgets.QPushButton, 'resetGyroButton')
        self.resetGyroButton.pressed.connect(lambda: self.buttonPressed("resetGyro"))

        self.rawCommandButton = self.findChild(QtWidgets.QPushButton, 'rawCommandButton')
        self.rawCommandButton.pressed.connect(self.sendRawCommands)

        self.safeLandingButton = self.findChild(QtWidgets.QPushButton, 'safeLandingButton')
        self.safeLandingButton.pressed.connect(lambda:self.buttonPressed("safeLanding"))

        self.connectionButton = self.findChild(QtWidgets.QPushButton, 'connectionButton')
        self.connectionButton.pressed.connect(self.connect)

        self.forwardButton = self.findChild(QtWidgets.QPushButton, 'forwardButton')
        self.forwardButton.pressed.connect(lambda:self.buttonPressed("f1"))
        self.forwardButton.released.connect(lambda:self.buttonReleased("f0"))

        self.leftButton = self.findChild(QtWidgets.QPushButton, 'leftButton')
        self.leftButton.pressed.connect(lambda:self.buttonPressed("l1"))
        self.leftButton.released.connect(lambda:self.buttonReleased("l0"))

        self.rightButton = self.findChild(QtWidgets.QPushButton, 'rightButton')
        self.rightButton.pressed.connect(lambda:self.buttonPressed("r1"))
        self.rightButton.released.connect(lambda:self.buttonReleased("r0"))

        self.backwardsButton = self.findChild(QtWidgets.QPushButton, 'backwardsButton')
        self.backwardsButton.pressed.connect(lambda:self.buttonPressed("b1"))
        self.backwardsButton.released.connect(lambda:self.buttonReleased("b0"))

        self.upButton = self.findChild(QtWidgets.QPushButton, 'upButton')
        self.upButton.pressed.connect(lambda:self.buttonPressed("u1"))
        self.upButton.released.connect(lambda:self.buttonReleased("u0"))

        self.downButton = self.findChild(QtWidgets.QPushButton, 'downButton')
        self.downButton.pressed.connect(lambda:self.buttonPressed("d1"))
        self.downButton.released.connect(lambda:self.buttonReleased("d0"))

        self.turnLeftButton = self.findChild(QtWidgets.QPushButton, 'turnLeftButton')
        self.turnLeftButton.pressed.connect(lambda:self.buttonPressed("tl1"))
        self.turnLeftButton.released.connect(lambda:self.buttonReleased("tl0"))

        self.turnRightButton = self.findChild(QtWidgets.QPushButton, 'turnRightButton')
        self.turnRightButton.pressed.connect(lambda:self.buttonPressed("tr1"))
        self.turnRightButton.released.connect(lambda:self.buttonReleased("tr0"))

        self.pidAxis.addItem("X")
        self.pidAxis.addItem("Y")

        self.pixmapWidth=400
        self.pixmapHeight=350
        xCanvas = QtGui.QPixmap(self.pixmapWidth, self.pixmapHeight)
        yCanvas = QtGui.QPixmap(self.pixmapWidth, self.pixmapHeight)
        zCanvas = QtGui.QPixmap(self.pixmapWidth, self.pixmapHeight)
        self.xGyroLabel.setPixmap(xCanvas)
        self.yGyroLabel.setPixmap(yCanvas)
        self.zGyroLabel.setPixmap(zCanvas)
        pen = QtGui.QPen()
        xPainter = QtGui.QPainter(self.xGyroLabel.pixmap())
        yPainter = QtGui.QPainter(self.yGyroLabel.pixmap())
        zPainter = QtGui.QPainter(self.zGyroLabel.pixmap())
        pen.setColor(QtGui.QColor('red'))
        xPainter.setPen(pen)
        yPainter.setPen(pen)
        zPainter.setPen(pen)
        xPainter.drawLine(0, 0, self.pixmapWidth, self.pixmapHeight)
        yPainter.drawLine(0, 0, self.pixmapWidth, self.pixmapHeight)
        zPainter.drawLine(0, 0, self.pixmapWidth, self.pixmapHeight)
        xPainter.end()
        yPainter.end()
        zPainter.end()
        self.drawXPosition(0)
        self.drawYPosition(0)
        self.drawZPosition(0)
        self.updatePowerBar(0)

        self.show()

    def updateCameraImage(self, img):
        self.cameraLabel.setPixmap(img)


    def on_key_pressed(self, key):
        try:
            if(key.char=='w' or key.char=='W'):
                self.updateCommand("f1")
            elif(key.char=='s' or key.char=='S'):
                self.updateCommand("b1")
            elif(key.char=='a' or key.char=='A'):
                self.updateCommand("l1")
            elif(key.char=='d' or key.char=='D'):
                self.updateCommand("r1")
            elif(key.char=='i' or key.char=='I'):
                self.updateCommand("u1")
            elif(key.char=='k' or key.char=='K'):
                self.updateCommand("d1")
            elif(key.char=='j' or key.char=='J'):
                self.updateCommand("tl1")
            elif(key.char=='l' or key.char=='L'):
                self.updateCommand("tr1")
        except:
            pass

    def on_key_release(self, key):
        try:
            if(not self.testingCheckBox.isChecked()):
                if(key.char=='w' or key.char=='W'):
                    self.updateCommand("f0")
                elif(key.char=='s' or key.char=='S'):
                    self.updateCommand("b0")
                elif(key.char=='a' or key.char=='A'):
                    self.updateCommand("l0")
                elif(key.char=='d' or key.char=='D'):
                    self.updateCommand("r0")
                elif(key.char=='i' or key.char=='I'):
                    self.updateCommand("u0")
                elif(key.char=='k' or key.char=='K'):
                    self.updateCommand("d0")
                elif(key.char=='j' or key.char=='J'):
                    self.updateCommand("tl0")
                elif(key.char=='l' or key.char=='L'):
                    self.updateCommand("tr0")
        except:
            if(key==keyboard.Key.esc):
                self.updateCommand("kill")

    def powerSliderChanged(self):
        self.updateCommand("PowerSlider "+str(self.powerSlider.value()))
        self.powerLabel.setText(str(self.powerSlider.value())+"%")

    def mpuSliderChanged(self):
        self.mpuLabel.setText(str(self.mpuComSlider.value())+"%")
        try:mpuComOff=float(self.mpuComOffEdit.text())
        except: mpuComOff=0.0
        try: mpuComMul=float(self.mpuComMulEdit.text())
        except: mpuComMul=1.0
        mpuVal=self.mpuComSlider.value()*mpuComMul+mpuComOff
        self.updateCommand("MPUSlider "+str(mpuVal))

    def pidSliderChanged(self, label, slider, offset, multiplier, type):
        sliderVal=slider.value()
        try: offsetNumber=float(offset.text())
        except: offsetNumber=0.0
        try: multiplierNumber=float(multiplier.text())
        except: multiplierNumber=1.0
        finalVal=offsetNumber+sliderVal*multiplierNumber
        self.updateCommand("PIDSlider pid " + self.pidAxis.currentText()+ " " +type+":"+str(finalVal))
        label.setText(str(sliderVal)+"%")

    def updateGyro(self, list):
        self.drawXPosition(list[0])
        self.drawYPosition(list[1])
        self.drawZPosition(list[2])

    def updatePowerBar(self, val):
        self.batteryLabel.setText(str(val))
        if val<=100 and val>=0:
            self.batteryLabel.setStyleSheet("background:qlineargradient(x:0, y0:0, x2:1, y2:0, stop: 0 rgb(255,0,0), stop: 1 rgb("+str(int(255*(100-val)/100))+","+str(int(255/100*val))+",0));")
        else:
            self.batteryLabel.setValue(str(0))

    def setPowerBar(self, value):
        self.batteryBar.setValue(value)

    def drawXPosition(self, val):
        self.drawXYPosition(self.xGyroLabel,val)

    def drawYPosition(self, val):
        self.drawXYPosition(self.yGyroLabel,val)

    def drawZPosition(self, val):
        painter = QtGui.QPainter(self.zGyroLabel.pixmap())
        painter.setBrush(QtGui.QBrush(Qt.black, Qt.SolidPattern))
        painter.drawRect(0, 0, self.pixmapWidth, self.pixmapHeight)
        pen = QtGui.QPen()
        pen.setWidth(18)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        c = self.pixmapWidth * 0.8
        newXVal = math.cos(val / 180 * math.pi) * c / 2
        newYVal = math.sin(val / 180 * math.pi) * c / 2
        midpoint = (int(self.pixmapWidth * 0.5), int(self.pixmapHeight * 0.5))
        leftOfMidpoint = (int(midpoint[0] - newXVal), int(midpoint[1] - newYVal))
        rightOfMidpoint = (int(midpoint[0] + newXVal), int(midpoint[1] + newYVal))
        arrowXVal = math.cos((val - 90) / 180 * math.pi) * c / 2
        arrowYVal = math.sin((val - 90) / 180 * math.pi) * c / 2
        arrowCoord = (int(midpoint[0] + arrowXVal), int(midpoint[1] + arrowYVal))

        painter.drawLine(midpoint[0], midpoint[1],
                         arrowCoord[0], arrowCoord[1])

        painter.drawLine(arrowCoord[0], arrowCoord[1],
                         int(arrowCoord[0] + (leftOfMidpoint[0] - arrowCoord[0]) * 0.3),
                         int(arrowCoord[1] + (leftOfMidpoint[1] - arrowCoord[1]) * 0.3))

        painter.drawLine(arrowCoord[0], arrowCoord[1],
                         int(arrowCoord[0] + (rightOfMidpoint[0] - arrowCoord[0]) * 0.3),
                         int(arrowCoord[1] + (rightOfMidpoint[1] - arrowCoord[1]) * 0.3))
        painter.end()
        self.update()

    def drawXYPosition(self, label, val):
        painter = QtGui.QPainter(label.pixmap())
        painter.setBrush(QtGui.QBrush(Qt.black, Qt.SolidPattern))
        painter.drawRect(0, 0, self.pixmapWidth, self.pixmapHeight)
        pen = QtGui.QPen()
        pen.setWidth(18)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        c=self.pixmapWidth*0.8
        newXVal=math.cos(val/180*math.pi)*c/2
        newYVal=math.sin(val/180*math.pi)*c/2
        midpoint=(int(self.pixmapWidth * 0.5), int(self.pixmapHeight * 0.5))
        leftOfMidpoint=(int(midpoint[0]-newXVal), int(midpoint[1]-newYVal))
        rightOfMidpoint = (int(midpoint[0] + newXVal), int(midpoint[1] + newYVal))
        arrowXVal = math.cos((val-90) / 180 * math.pi) * c / 2
        arrowYVal = math.sin((val-90) / 180 * math.pi) * c / 2
        arrowCoord = (int(midpoint[0]+arrowXVal), int(midpoint[1]+arrowYVal))
        painter.drawLine(leftOfMidpoint[0], leftOfMidpoint[1],
                         rightOfMidpoint[0], rightOfMidpoint[1])

        painter.drawLine(midpoint[0], midpoint[1],
                         arrowCoord[0], arrowCoord[1])

        painter.drawLine(arrowCoord[0], arrowCoord[1],
                         int(arrowCoord[0]+(leftOfMidpoint[0]-arrowCoord[0])*0.3), int(arrowCoord[1]+(leftOfMidpoint[1]-arrowCoord[1])*0.3))

        painter.drawLine(arrowCoord[0], arrowCoord[1],
                         int(arrowCoord[0]+(rightOfMidpoint[0]-arrowCoord[0])*0.3), int(arrowCoord[1]+(rightOfMidpoint[1]-arrowCoord[1])*0.3))
        painter.end()
        self.update()

    def buttonPressed(self, command):
        self.mainProgram.sendMessage(command)

    def buttonReleased(self,command):
        self.updateCommand(command)

    def updateCommand(self, command):
        self.mainProgram.processCommand(command)

    def searchForAddresses(self):
        self.mainProgram.netScan()

    def updateGui(self):
        self.fillIpAddresses()

    def updateConnectButton(self):
        self.connectionButton.setText(self.mainProgram.getConnectionState(self.connectionButton.text()))

    def sendRawCommands(self):
        self.mainProgram.sendMessage(self.manualCommand.text())
        

    def fillIpAddresses(self):
        self.addressComboBox.clear()
        for IP in self.mainProgram.foundIPs:
            self.addressComboBox.addItem(IP)

    def connect(self):
        if self.connectionButton.text()=="Connect":
            self.mainProgram.connect(self.addressComboBox.currentText())
        else:
            self.mainProgram.disconnect()

app = QtWidgets.QApplication(sys.argv)
gui=Ui()
app.exec_()
gui.listener.stop()
