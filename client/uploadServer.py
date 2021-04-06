import os
#os.system('cmd /c scp droneServer/server.py pi@192.168.137.245:~/newDrone')
import paramiko
from scp import SCPClient

def createSSHClient(server, port, user, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(server, port, user, password)
    return client

password=input("Password?")
ssh = createSSHClient("192.168.137.245", "22", "pi", password)
scp = SCPClient(ssh.get_transport())
scp.put("../droneServer",recursive=True, remote_path="~/newDrone")
#stdin, stdout, stderr = ssh.exec_command("sudo python3 ~/newDrone/server.py")
#stdin, stdout, stderr = ssh.exec_command("ls", get_pty=True)
#for line in stdout.readline(), "":
#    print(line, end="")

ssh.close()