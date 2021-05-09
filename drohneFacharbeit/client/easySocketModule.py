import socket
import threading
import time
from socket import getaddrinfo, AF_INET, gethostname
import netifaces

class NetworkScanner:
    def __init__(self):
        self.DefaultPort=5005
        self.IP=socket.gethostbyname(socket.gethostname())
        self.niInterfaces=netifaces.interfaces()

    def getNames(self):
        list=[]
        for niI in self.niInterfaces:
            for link in netifaces.ifaddresses(niI)[netifaces.AF_INET]:
                list+=[link["addr"]]
        return list

    def checkIP(self,IP, DefaultPort):
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        if (s.connect_ex((IP, DefaultPort))==0):
            return True
        s.close()

    def scan(self):
        addresses=[]
        for ip in self.getNames():
            var=ip.split(".")
            i=1
            while (i<256):
                adr=var[0]+"."+var[1]+"."+var[2]+"."+str(i)
                s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                socket.setdefaulttimeout(0.05)
                if(s.connect_ex((adr, 5005))==0):
                    addresses+=[adr]
                s.close()
                i+=1
        return addresses

class EasyUDPSocket:
    def f(self):
        pass
    def __init__(self, bufferSize=1024, targetIP="", ownPort=5006, targetPort=5005, receivingActive=True, f=f):
        self.__sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.__targetIP=targetIP
        self.__targetPort=targetPort
        self.__bufferSize=bufferSize
        self.__ended=False
        self.__receivingPaused=False
        self.__ownPort=ownPort
        self.__f=f
        if receivingActive:
            threading.Thread(target=self.receiveMessages).start()

    def bind(self):
        self.__sock.bind(("",self.__ownPort))

    def pause(self):
        self.__receivingPaused=False

    def resume(self):
        self.__receivingPaused=True

    def sendMessage(self, message):
        x=lambda : self.__sock.sendto(message.encode(), (self.__targetIP, self.__targetPort))
        threading.Thread(target=x).start()

    def sendto(self,data):
        self.__sock.sendto(data,(self.__targetIP, self.__targetPort))

    def closeSocket(self):
        self.__ended=True
        self.__sock.close()

    def recv(self):
        return self.__sock.recvfrom(self.__bufferSize)

    def receiveMessages(self):
        self.__sock.bind(("", self.__ownPort))
        while not self.__ended:
            if not self.__receivingPaused:
                data, addr=self.__sock.recv(self.__bufferSize)
                self.__f(data.decode(), addr)

    def setReceiveFunction(self, f):
        self.__f=f

class EasyTCPClient:
    def f(self, x, y):
        print(x,y)

    def __init__(self, bufferSize=1024, targetIP="localhost", port=5005, receivingActive=True, f=f):
        self.__sock=socket.socket()
        self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__targetIP=targetIP
        self.__targetPort=port
        self.__bufferSize=bufferSize
        self.__ended=False
        self.__connected=False
        self.__receivingPaused=False
        self.__f=f
        if receivingActive:
            threading.Thread(target=self.receiveMessages).start()

    def connect(self):
        try:
            #self.__sock.connect(("192.168.2.36", 5005))
            self.__sock.connect((self.__targetIP, self.__targetPort))
            self.__connected=True
        except Exception as e: print(e)

    def disconnect(self):
        try:
            self.closeSocket()
        except: pass

    def isConnected(self):
        return True if self.__connected else False

    def sendMessage(self, message):
        try:
            if self.__connected:
                x=lambda : self.__sock.send(message.encode())
                threading.Thread(target=x).start()
        except: self.__connected=False

    def sendall(self, data):
        self.__sock.sendall(data)

    def recv(self):
        return self.__sock.recv(self.__bufferSize)

    def closeSocket(self):
        self.__ended=True
        self.__connected=False
        self.__sock.close()

    def receiveMessages(self):
        try:
            while not self.__ended:
                if not self.__receivingPaused and self.__connected:
                    data=self.__sock.recv(self.__bufferSize).decode()
                    self.__f(data)
        except Exception as e:
            print(e)
            self.__connected=False
            self.closeSocket()

    def setReceiveFunction(self, f):
        self.__f=f

class Client:
    def f(self, data):
        print(data)

    def __init__(self, server, con, addr, buffersize=1024):
        self.__connection=con
        self.__addr=addr
        self.__server=server
        self.__bufferzize=buffersize

    def sendto(self, message):
        self.__connection.send(message.encode())

    def evaluate(self, data):
        pass

    def handle(self):
        while not self.__server.ended:
            try:
                data=self.__connection.recv(self.__bufferzize).decode()
            except:
                print("connection lost:", self.__addr)
                self.__server.removeClient(self.__server.getClients().index(self))
            self.evaluate(data)

    def closeConnection(self):
        self.__connection.close()
        print("connection closed:", self.__addr)
        self.__server.removeClient(self.__server.getClients().index(self))

    def recv(self):
        return self.__sock.recv(self.__bufferSize)

class EasyTCPServerMultiClient:
    def __init__(self, bufferSize=1024, port=5005, acceptWord=""):
        self.__sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.__sock.bind(("", 5005))
        self.__sock.listen(0)
        self.__port=port
        self.__bufferSize=bufferSize
        self.__acceptWord=acceptWord
        self.ended=False
        self.__clients=[]
        threading.Thread(target=self.acceptConnections).start()

    def getClients(self):
        return self.__clients

    def removeClient(self, index):
        self.__clients=self.__clients[:index]+self.__clients[index+1:]

    def acceptConnections(self):
        while not self.ended:
            con, addr=self.__sock.accept()
            if self.__acceptWord!="":
                data=con.recv(self.__bufferSize).decode()
                if data==self.__acceptWord:
                    self.__clients+=[Client(self, con, addr, self.__bufferSize)]
                    threading.Thread(target=newClient.handle).start()
            else:
                newClient=Client(self, con, addr, self.__bufferSize)
                self.__clients+=[newClient]
                threading.Thread(target=newClient.handle).start()

    def closeSocket(self):
        self.ended=True
        self.__sock.close()

class EasyTCPServerSingleClient:
    def __init__(self, bufferSize=1024, port=5005, acceptWord="", autonomous=False, f=lambda:print("-")):
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__sock.bind(("", 5005))
        self.__sock.listen(0)
        self.__port = port
        self.__bufferSize = bufferSize
        self.__acceptWord = acceptWord
        self.ended = False
        if autonomous:
            threading.Thread(target=self.run).start()

    def sendall(self, data):
        self.__sock.sendall(data)

    def recv(self):
        return self.__sock.recv(self.__bufferSize)

    def accept(self):
        data= self.__sock.accept()
        return data

    def closeSocket(self):
        self.__sock.close()

    def run(self):
        while not self.ended:
            con, addr=self.__sock.accept()
            if self.__acceptWord:
                try:
                    if not self.__sock.recv(self.__bufferSize)==self.__acceptWord:
                        con.close()
                except: con.close()
            self.__f(con,addr)

if __name__=="__main__":
    """class Client(Client):
        def __init__(self, server, con, addr, buffersize):
            super(Client, self).__init__(server, con, addr, buffersize)
        def evaluate(self, data):
            print(data)"""

    def udpTest():
        f=lambda x,y: print(x+", "+str(y))
        u1=EasyUDPSocket(f=f)
        u2=EasyUDPSocket(f=f,targetIP="", targetPort=5006, ownPort=5005)
        time.sleep(5)
        u1.sendMessage("Hallo")
        u1.closeSocket()
        u2.closeSocket()

    def tcpTest(client=Client):
        server=EasyTCPServer()
        client=EasyTCPClient()
        time.sleep(4)
        client.connect()
    #udpTest()
    #server=easySocketModule(mode="server")
    nn=NetworkScanner()
    print(nn.getNames())
    nn.scan()
#client=easySocketModule(adr="192.168.2.36")
