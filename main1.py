import machine
from machine import SoftI2C, Pin, ADC
import pca9685
import time
import uasyncio
    

#This function is written to use with adafruit motor shield to control brush motor
#PWM is PCA9685 object
def motor(pwm, index, speed):
    
    if (index<1 or index>4):
        return

    #pwm 10,11,4,5 connected to respective IN1 (TB6612FNG)
    #pwm 9,12,3,6 connected to respective IN2 (TB6612FNG)
    #pwm 8,13,2,7 connected to respective PWM

    #Motor1 controled by pwm8,9,10
    if (index==1):
        in1 = 10
        in2 = 9
        pwin = 8
    #Motor2 controled by pwm11,12,13
    elif(index==2):
        in1 = 11
        in2 = 12
        pwin = 13
    #Motor3 controled by pwm2,3,4
    elif(index==3):
        in1 = 4
        in2 = 3
        pwin = 2        
    #Motor4 controled by pwm5,6,7
    elif(index==4):
        in1 = 5
        in2 = 6
        pwin = 7
        
    #reverse direction
    if (speed>0):
        pwm.duty(in1, 0)
        pwm.duty(in2, 4095)
    else:
        pwm.duty(in1, 4095)
        pwm.duty(in2, 0)
        speed = 0-speed

    pwm.duty(pwin, speed)
    

#Initialize i2C on ESPDUNO (using soft i2c)
i2c = SoftI2C(scl=machine.Pin(22), sda=machine.Pin(21), freq=100000)
#Attach i2c to PCA9865
pwm = pca9685.PCA9685(i2c, address=0x60)
#Setup PWM frequency
pwm.freq(40000)

while True:
    for i in range (5):
        motor(pwm, 3, i*1000)


