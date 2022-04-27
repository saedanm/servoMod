import machine
from machine import SoftI2C, Pin, ADC
import pca9685
import time
import uasyncio

gain_kp = 2.5
gain_kd = 5.0
pot_adc_desired  = 0
pot_adc_actual   = 0
total_pot_adc    = 0
pat_sample_count = 0

motor_channel = 1
pot_adc_channel = 39

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
    
    
#---------- Position control loop
def scale_value(value, in_min, in_max, out_min, out_max):
  scaled_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
  return scaled_value

err=0
derr = 0
prev_err = 0
def control_loop(timer):
    global pot_adc_desired
    global pot_adc_actual
    global gain_kp
    global gain_kd
    global pwm
    global err
    global derr
    global prev_err
    global motor_channel
    #Obtain desired motor position (angle)   
    #Read actual potentiometer voltage in ADC number
    #Compute error
    err = pot_adc_desired - pot_adc_actual
    
    derr = err-prev_err
    
    prev_err = err
    
    #Compute control signal
    out = gain_kp*err + gain_kd*derr
    
    if (out<-1000):
        out=-1000
    
    if (out>1000):
        out=1000
    
    #Map to PWM
    pwm_val = scale_value(out, -1000, 1000, -4000, 4000)
    
    #Sent PWM out to motor
    motor(pwm, motor_channel, int(pwm_val))


def sampling_loop(timer):
    global pot_adc_actual
    global total_pot_adc
    global pat_sample_count
    
    total_pot_adc = total_pot_adc+potentiometer.read()
    
    pat_sample_count = pat_sample_count+1
    
    if (pat_sample_count >= 8):
        pot_adc_actual = total_pot_adc>>3
        total_pot_adc = 0
        pat_sample_count = 0    
            
    
#----------- blinking built-in LED with timer0
led_toggle = False
def blink_led(timer):
    global led_toggle
    led_toggle = not led_toggle
    if (led_toggle==True):
        led.on()
    else:
        led.off()

#Set up built-in LED for blinking
led = Pin(2, Pin.OUT)
tim0 = machine.Timer(0)
tim0.init(period=1000,mode=machine.Timer.PERIODIC, callback=blink_led)

potentiometer = ADC(Pin(pot_adc_channel))            #creating potentiometer object
potentiometer.atten(ADC.ATTN_11DB)       #3.3V full range of voltage
pot_adc_actual = potentiometer.read()

#Let set pot_adc_desired for tim1.initthis time
pot_adc_desired = 1500

#Initialize i2C on ESPDUNO (using soft i2c)
i2c = SoftI2C(scl=machine.Pin(22), sda=machine.Pin(21), freq=100000)
#Attach i2c to PCA9865
pwm = pca9685.PCA9685(i2c, address=0x60)
#Setup PWM frequency
pwm.freq(40000)


control_freq = 500
#Setup timer to perform position control
tim1 = machine.Timer(1)
#Running control loop at 1 kHz
tim1.init(freq=control_freq,mode=machine.Timer.PERIODIC, callback=control_loop)

#Setup timer to perform position control
tim2 = machine.Timer(2)
#Maximum sampling rate for ADC is 6 kHz
tim2.init(freq=8*control_freq,mode=machine.Timer.PERIODIC, callback=sampling_loop)

toggle = True
while True:
    if (toggle):
        toggle=False
        pot_adc_desired = 1500
        time.sleep(1)
    else:
        toggle=True
        pot_adc_desired = 3500        
        time.sleep(1)
    
