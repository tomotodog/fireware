'''
Copyright Deng (ream_d@yeah.net)  Py-apple dog project

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''
#引入模块
import webrepl
from math import *
from machine import I2C
from machine import UART
from machine import Pin
import time
import _thread


#链接网络，启用WEBREPL
def do_connect(essid, password):
    import network 
    wifi = network.WLAN(network.STA_IF)  
    if not wifi.isconnected(): 
        print('connecting to network...')
        wifi.active(True) 
        wifi.connect(essid, password) 
        while not wifi.isconnected():
            pass 
    print('network config:', wifi.ifconfig())
    
do_connect('ESP_TEST','12345678')  #账号密码

webrepl.start() 


#主程序开始
#==============版本信息==================#
version="V1.2 BETA 1 2020516"

#打印版本信息
print("=============================")
print("PY-APPLE DOG TEST VER "+version)
print("=============================")
print("作者：灯哥  ream_d@yeah.net    开源协议：Apache License")
print("=========实现功能=========")
print("1、踏步 2、高度调节 3、小跑基础版（向前、向后）")
print("=========实现功能=========")
print("加载程序...")


#=============默认速度 步长 设置=============
l1=55   #大腿长(mm)
l2=56   #小腿长(mm)
h=50    #抬腿高度 设置
xf=30
xs=-10
#xf=0
#xs=0
Ts=1   #周期
faai=0.5   #占空比


speed=0.1  #步频调节



#=============默认舵机中位值设置=============


#狗腿子顺序(从1开始顺时针数)


#===头===


#1==-==2


#4==-==3

init_1k = 99#45   #1脚胯部
init_1h = 85#45   #1脚大腿
init_1s = 89#132   #1脚小腿

init_2k = 101#45   #2脚胯部
init_2h = 88#122   #2脚大腿
init_2s = 113#54 #2脚小腿

init_3k = 95#45   #3脚胯部
init_3h = 83#120  #3脚大腿
init_3s = 109#54   #3脚小腿

init_4k = 97#45   #4脚胯部
init_4h = 89#45   #4脚大腿
init_4s = 109#144   #4脚小腿

#=============高度调节比例系数=============
Kp_H=0.03
#=============一些中间变量=============
t=0
init_x=0
init_y=-100
x1=init_x
y1=init_y
x2=init_x
 
y2=init_y
x3=init_x
y3=init_y
x4=init_x
y4=init_y

ges_x_1=0
ges_x_2=0
ges_x_3=0
ges_x_4=0

ges_y_1=init_y
ges_y_2=init_y
ges_y_3=init_y
ges_y_4=init_y

import ustruct
import time


class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()

    def _write(self, address, value):
        self.i2c.writeto_mem(self.address, address, bytearray([value]))

    def _read(self, address):
        return self.i2c.readfrom_mem(self.address, address, 1)[0]

    def reset(self):
        self._write(0x00, 0x00) # Mode1

    def freq(self, freq=None):

        if freq is None:
            return int(25000000.0 / 4096 / (self._read(0xfe) - 0.5))
        prescale = int(25000000.0 / 4096.0 / freq + 0.5)
        old_mode = self._read(0x00) # Mode 1
        self._write(0x00, (old_mode & 0x7F) | 0x10) # Mode 1, sleep
        self._write(0xfe, prescale) # Prescale
        self._write(0x00, old_mode) # Mode 1
        time.sleep_us(5)
        self._write(0x00, old_mode | 0xa1) # Mode 1, autoincrement on

    def pwm(self, index, on=None, off=None):
        if on is None or off is None:
            data = self.i2c.readfrom_mem(self.address, 0x06 + 4 * index, 4)
            return ustruct.unpack('<HH', data)
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, 0x06 + 4 * index,  data)

    def duty(self, index, value=None, invert=False):
        if value is None:
            pwm = self.pwm(index)
            if pwm == (0, 4096):
                value = 0
            elif pwm == (4096, 0):
                value = 4095
            value = pwm[1]
            if invert:
                value = 4095 - value
            return value
        if not 0 <= value <= 4095:
            raise ValueError("Out of range")
        if invert:
            value = 4095 - value
        if value == 0:
            self.pwm(index, 0, 4096)
        elif value == 4095:
            self.pwm(index, 4096, 0)
        else:
            self.pwm(index, 0, value)


import math

class Servos:
    def __init__(self, i2c, address=0x40, freq=50, min_us=500, max_us=2500,  #根据舵机参数自行设置
                 degrees=180):
        self.period = 1000000 / freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.pca9685 = PCA9685(i2c, address)
        self.pca9685.freq(freq)

    def _us2duty(self, value):
        return int(4095 * value / self.period)

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        self.pca9685.duty(index, 0)

    def position_duty(self, index, degrees=None, radians=None, us=None, duty=None):
        int_dutu=int(duty)
        self.pca9685.duty(index, int_dutu)


servos = Servos(I2C(scl=Pin(23), sda=Pin(19), freq=100000), address=0x40)#舵机控制板

def init_servo_mov():
    global init_1h,init_1s,init_2h,init_2s,init_3h,init_3s,init_4h,init_4s, flag
    #print(init_1s)
    if flag == 1:
        #腿1
        servos.position(0, degrees=init_1k)
        servos.position(4, degrees=init_1h)  # 腿1大腿
        servos.position(5, degrees=init_1s)  # 腿1小腿
        #腿2
        servos.position(1, degrees=init_2k)
        servos.position(6, degrees=init_2h)  # 腿2大腿
        servos.position(7, degrees=init_2s)  # 腿2小腿
        #腿3
        servos.position(2, degrees=init_3k)
        servos.position(8, degrees=init_3h)  # 腿3大腿
        servos.position(9, degrees=init_3s)  # 腿3小腿
        #腿4
        servos.position(3, degrees=init_4k)
        servos.position(10, degrees=init_4h)  # 腿4大腿
        servos.position(11, degrees=init_4s)  # 腿4小腿
    else :
        servos.position(0, degrees=init_1k)
        servos.position(4, degrees=90)  # 腿1大腿
        servos.position(5, degrees=90)  # 腿1小腿
        #腿2
        servos.position(1, degrees=init_2k)
        servos.position(6, degrees=90)  # 腿2大腿
        servos.position(7, degrees=90)  # 腿2小腿
        #腿3
        servos.position(2, degrees=init_3k)
        servos.position(8, degrees=90)  # 腿3大腿
        servos.position(9, degrees=90)  # 腿3小腿
        #腿4
        servos.position(3, degrees=init_4k)
        servos.position(10, degrees=90)  # 腿4大腿
        servos.position(11, degrees=90)  # 腿4小腿
        flag =1
#===头===
#1==-==2
#4==-==3
def foot_init():


    global init_1h,init_1s,init_2h,init_2s,init_3h,init_3s,init_4h,init_4s,flag

    flag = 1
    print("PY-APPLE DOG 菠萝狗辅助脚调中系统")


    print("==================================")


    init_servo_mov()


    while True:


        print("#方向")


        print("#===头===")


        print("#1==-==2")


        print("#4==-==3")


        user_leg_num=input("请输入腿号【按q并回车退出调平,按z设置每个舵机安装角度90度】>>")


        if user_leg_num=="q":


            print("=============调中结果=============")

            print("init_1k:",init_1k)
            print("init_1h:",init_1h)
            print("init_1s:",init_1s)
            print("init_2k:",init_2k)
            print("init_2h:",init_2h)
            print("init_2s:",init_2s)
            print("init_3k:",init_3k)
            print("init_3h:",init_3h)
            print("init_3s:",init_3s)
            print("init_4k:",init_4k)
            print("init_4h:",init_4h)
            print("init_4s:",init_4s)
            print("=============调中结果=============")

            break
        
        elif  user_leg_num=="z":
            flag = 0
            init_servo_mov()
        else :

            while True:
                init_servo_mov()
                user_leg_die=input("请输入调平方向并回车【+/-/q(退出)】>>")
                if user_leg_die=="+":
                    exec("init_"+user_leg_num+"="+"init_"+user_leg_num+"+1")
                elif user_leg_die=="-":
                    exec("init_"+user_leg_num+"="+"init_"+user_leg_num+"-1")

                elif user_leg_die=="q":
                    break

def servo_output():
    #腿1
    servos.position(0, degrees=99)
    servos.position(4, degrees=init_1h-ham1)  # 腿1大腿
    servos.position(5, degrees=(init_1s)+1*(90-kip1))  # 腿1小腿
    #腿2
    servos.position(1, degrees=101)
    servos.position(6, degrees=init_2h+ham2)  # 腿2大腿
    servos.position(7, degrees=(init_2s)-1*(90-kip2))  # 腿2小腿
    #腿3
    servos.position(2, degrees=95)
    servos.position(8, degrees=init_3h+ham3)  # 腿3大腿
    servos.position(9, degrees=(init_3s)-1*(90-kip3))  # 腿3小腿
    #腿4
    servos.position(3, degrees=97)
    servos.position(10, degrees=init_4h-ham4)  # 腿4大腿
    servos.position(11, degrees=(init_4s)+1*(90-kip4))  # 腿4小腿


def caculate():
    print("Caculating...")
    global x1,y1,x2,y2,x3,y3,x4,y4
    global kip1,kip2,kip3,kip4
    global ham1,ham2,ham3,ham4
    global shank1,shank2,shank3,shank4
    #腿1
    x1=-x1
    shank1=pi-acos((x1*x1+y1*y1-l1*l1-l2*l2)/(-2*l1*l2))
    fai1=acos((l1*l1+x1*x1+y1*y1-l2*l2)/(2*l1*sqrt(x1*x1+y1*y1)))
    if x1>0:
        ham1=abs(atan(y1/x1))-fai1
    elif x1<0:
        ham1=pi-abs(atan(y1/x1))-fai1
    else:
        ham1=pi-1.5707-fai1
    shank1=180*shank1/pi
    ham1=180*ham1/pi
    kip1=(180-ham1-shank1)
    print('kip1:',kip1)
    print('ham1:',ham1)
    print('shank1:',shank1)
    
    #腿2
    x2=-x2
    shank2=pi-acos((x2*x2+y2*y2-l1*l1-l2*l2)/(-2*l1*l2))

    fai2=acos((l1*l1+x2*x2+y2*y2-l2*l2)/(2*l1*sqrt(x2*x2+y2*y2)))
    if x2>0:
        ham2=abs(atan(y2/x2))-fai2
    elif x2<0:
        ham2=pi-abs(atan(y2/x2))-fai2
    else:
        ham2=pi-1.5707-fai2
    shank2=180*shank2/pi
    ham2=180*ham2/pi
    kip2=(180-ham2-shank2)
    print('kip2:',kip2)
    print('ham2:',ham2)
    print('shank2:',shank2)
    
    #腿3
    x3=-x3
    shank3=pi-acos((x3*x3+y3*y3-l1*l1-l2*l2)/(-2*l1*l2))
    fail3=acos((l1*l1+x3*x3+y3*y3-l2*l2)/(2*l1*sqrt(x3*x3+y3*y3)))
    if x3>0:
        ham3=abs(atan(y3/x3))-fail3
    elif x3<0:
        ham3=pi-abs(atan(y3/x3))-fail3
    else:
        ham3=pi-1.5707-fail3
    shank3=180*shank3/pi
    ham3=180*ham3/pi
    kip3=(180-ham3-shank3)
    print('kip3:',kip3)
    print('ham3:',ham3)
    print('shank3:',shank3)
    
    #腿4
    x4=-x4
    shank4=pi-acos((x4*x4+y4*y4-l1*l1-l2*l2)/(-2*l1*l2))
    fai4=acos((l1*l1+x4*x4+y4*y4-l2*l2)/(2*l1*sqrt(x4*x4+y4*y4)))
    if x4>0:
        ham4=abs(atan(y4/x4))-fai4
    elif x4<0:
        ham4=pi-abs(atan(y4/x4))-fai4
    else:
        ham4=pi-1.5707-fai4
    shank4=180*shank4/pi
    ham4=180*ham4/pi
    kip4=(180-ham4-shank4)
    print('kip4:',kip4)
    print('ham4:',ham4)
    print('shank4:',shank4)
   

#===头===
#1==-==2
#4==-==3
#=======================小跑步态
def run_trot(r1,r4,r2,r3):    #小跑步态执行函数
    global t
    global x1,x2,x3,x4
    global y1,y2,y3,y4
    if t<=Ts*faai:
        sigma=2*pi*t/(faai*Ts)
        zep=h*(1-cos(sigma))/2
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs;
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        y1=ges_y_1+zep
        y2=ges_y_2
        y3=ges_y_3+zep
        y4=ges_y_4
        #输出x
        x1=-xep_z*r1
        x2=-xep_b*r2
        x3=-xep_z*r3
        x4=-xep_b*r4
    if t>Ts*faai and t<Ts:
        sigma=2*pi*(t-Ts*faai)/(faai*Ts)
        
        zep=h*(1-cos(sigma))/2;
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs;
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        y1=ges_y_1
        y2=ges_y_2+zep
        y3=ges_y_3
        y4=ges_y_4+zep
        #输出x
        x1=-xep_b*r1
        x2=-xep_z*r2
        x3=-xep_b*r3
        x4=-xep_z*r4
        #x1=0
        #x2=0
        #x3=0
        #x4=0
    print('=========')
    print('x1:',x1)
    print('x2:',x2)
    print('x3:',x3)
    print('x4:',x4)
    
    caculate()
    servo_output()
  
   
def trot(num):
    global t
    global x1,x2,x3,x4
    global y1,y2,y3,y4
    global xf,xs
    global ges_y_1,ges_y_4
    for i in range(abs(num)):
            while True: 

                if t>=Ts:#一个完整的运动周期结束
                    t=0
                    break
                else:
                    t=t+speed
                    if num>0:
                        run_trot(1,1,1,1)
                        xf=40
                        xs=-10
                        #ges_y_1=-100
                        #ges_y_4=-100
                    elif num<0:
                        xf=20
                        xs=-20
                        #ges_y_1=-110


                        #ges_y_4=-110


                        run_trot(-1,-1,-1,-1)


            print("-------------循环"+str(i+1)+"-------------")



    x1=0


    x2=0


    x3=0


    x4=0


    caculate()


    servo_output()


R_H=100


#高度调节函数


def height(goal):


    global y1,y2,y3,y4


    global x1,x2,x3,x4


    global R_H


    global ges_y_1,ges_y_2,ges_y_3,ges_y_4
    while True:
        if R_H>goal:
            R_H=R_H-abs(R_H-goal)*Kp_H
        elif R_H<goal:
            R_H=R_H+abs(R_H-goal)*Kp_H
        if abs(R_H-goal)<1:
            break
        y1 = -R_H

        y2 = -R_H
        y3 = -R_H
        y4 = -R_H
        
        ges_y_1 = -R_H
        ges_y_2 = -R_H
        ges_y_3 = -R_H
        ges_y_4 = -R_H
        x1=0
        x2=0
        x3=0
        x4=0

    
        caculate()
        servo_output()

#foot_init()
        
#trot(5)


height(100)       #调节高度到110mm
height(50)        #调节高度到50mm
height(100)       #调节高度到110mm
height(50)        #调节高度到50mm
height(100)       #调节高度到110mm
height(50)        #调节高度到50mm
height(80)       #调节高度到110mm
time.sleep(1)           #延时等待

trot(5)           #往前进5步
time.sleep(1)           #延时等待
trot(-5)          #往后退5步
time.sleep(1)           #延时等待
height(80)        #调节高度到50mm



