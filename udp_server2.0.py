#This program runs in raspberrypi,
#It can recieve messages using UDP protocol got from topside computer,and after data process send to STM32.
#Meanwhile, it can receive data from STM32 using UASRT and send to topside computer.
#Using two threads,and dont bother each other

#!/bin/env/python
#import necessary modules
#import serial
import time
import threading
import socket
import opengen as og
time.sleep(1)

#UDP Socket setup:
#server:Raspberry IP
#host='192.168.137.10'
host='10.25.202.138'
port=10000
#client: Topside Computer's address, set as global variable,shared in threads
global addr
#addr='192.168.137.1' 
addr='10.24.157.42'
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) #允许重用本地地址和端口，允许绑定已经被使用的地址或端口号
sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1) #允许发送广播数据，允许socket广播讯息到网络上
#bind the port
sock.bind((host, port))
#sock.connect((host,port))

#define const variable
BI1 = 0.9783
BI2 = 2.7174
BI3 = 0.9533

#Serial setup
#ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',115200)
#if ser.isOpen() == False:
#    ser.open()

#Thread 1 recieve messages using UDP protocol got from topside computer,and after data process send to STM32
#线程1实现树苺派接受Computer's UDP数据，并通过串口下发给单片机    
class MyThread01(threading.Thread): #This class is derived from the class threading.Thread
    def __init__(self, n):
        threading.Thread.__init__(self)
        self.n = n  
    def run(self):
        while True:
                try:
                    print("Tring to connect client")
                    sock.sendto(b'1',('10.24.157.42',10000))#send data
                    data,addr=sock.recvfrom(1024) #receive data from client
                    print("got data from",addr)
                    #print(data)#data type is bytes
                    #data type change from bytes to str
                    strdata=data.decode()
                    #print(type(strdata))
                    #print(strdata)
                    #judge the data is direct velocities or angle data
                    if(strdata[-1]==';'): #direct velocities data of eight motors end with ';'
                        bdata= strdata.encode()
                        #print(bdata)
                        print('got motor velocity data success')
                        #send data through uart
                        #ser.write(bdata)
                    #if is angle data,call NMPC of Opengen algrithm and data process
                    else:
                        #split the str array and change to list datatype
                        ldata=strdata.split(',')
                        #print(ldata)
                        #print(type(ldata)) #list
                        
                        #Call the Opengen solver, return the u
                        #tcp_config = og.config.TcpServerConfiguration(ip, port)
                        solver_status = mng.call(ldata,initial_guess=[1.0] * (3*50))
                        #print(solver_status)
                        us = solver_status['solution']
                        #print(us)
                        #print(type(us))
                        u = us[0:3]
                        #using fomula to calculate 8-axis thrust
                        thrusts = [BI1 * u[0] + BI2 * u[1],
                                    -BI1 * u[0] + BI2 * u[1],
                                    BI1 * u[0] + -BI2 * u[1],
                                    -BI1 * u[0] + -BI2 * u[1],
                                    -BI3 * u[2],
                                    BI3 * u[2],
                                    BI3 * u[2],
                                    -BI3 * u[2]]
                        print(thrusts) #thrusts is a list with each item belongs to float datatype
                        #print(type(thruster[0]))#float
                        #using fomula to converge thrust to velocities
                        vthrusts = []
                        for thrust in thrusts:
                            if thrust > 0:
                                x = 60 * pow(thrust/0.039501,0.5)
                                vthrusts.append(x)
                            elif thrust < 0:
                                x = -60 * pow(abs(thrust)/0.039501,0.5)
                                vthrusts.append(x)
                        #print(vthrusts)
                        #vthrusts is a list within each datatype is float
                        #float data change into int 
                        #thru_int = list(map(int,vthrusts))
                        thru_int = [int(a) for a in vthrusts]
                        #print(thru_int)
                        #int data change into string
                        strl_thru = [str(x) for x in thru_int]
                        #print(str_thru)
                        #list change into string
                        str_thru = ",".join(strl_thru)
                        str_thrust = str_thru+",;"
                        #string change into bytes
                        bstr_thru = str_thrust.encode(encoding="utf-8")
                        print(bstr_thru)
                        #send data through uart
                        #ser.write(bstr_thru)
                        #strdata = data.encode(encoding='utf-8')
                except  KeyboardInterrupt: #keyboard interrupt
                    raise #trigger exception
                
#Thread 2 receive data from STM32 using UASRT and send to topside computer               
#线程2树苺派将串口收到的数据通过UDP发送给电脑
class MyThread02(threading.Thread): 
    def __init__(self, n):
        threading.Thread.__init__(self)
        self.n = n                
    def run(self):
        while True:
            try:
                re = ser.readline() #receive serial data
                #print(type(re)) #bytes
                #re = ser.inWaiting()
                #rec = re.read(20)
                #recv = re.decode('UTF-8')
                #print(type(recv)) #type str
                #print(recv)
                #c=''.join(re) #list change tostring
                #print(type(c))
                #print(c)
                #print(addr)#client ip address
                #sendto()第一个参数是str，而readlines()返回值类型是List,所以要将list转换为str再发送
                #send bytes,no need to transfer to string is ok
                #print("receive data from IMU")
                #print(re)
                #send IMU data to topside computer
                sock.sendto(re,(addr,10000))
                #print(addr)
                #print("s")
                #time.sleep(0.1)    
            except:
                pass #do nothing 保持结构和语义完整
                #time.sleep(self.interval)

if __name__ == '__main__':
    #call opengen--control algorithm
    #mng = og.tcp.OptimizerTcpManager('rov_optimizers_test1/navigation1','10.25.202.138',9555)
    mng = og.tcp.OptimizerTcpManager('rov_optimizers_test1/navigation1')
    
    #start the TCP server
    mng.start()
    
    #start two threads
    thread1 = MyThread01('t1') #UDP recev serial send
    #thread2 = MyThread02('t2') #UDP send serial recev
    thread1.start()
    #thread2.start()
    
    #kill the TCP server
    mng.kill()
    
    time.sleep(1)
    #thread1.stop()
    #thread2.stop()
