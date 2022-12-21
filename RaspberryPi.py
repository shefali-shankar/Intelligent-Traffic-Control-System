import os
import time
import RPi.GPIO as gpio
import RPi.GPIO as GPIO                  
import time                                
GPIO.setmode(GPIO.BCM)


IN1=21
IN2=20
IN3=16
IN4=12

RED1 = 26                                  
GREEN1= 19

RED2 = 13                                  
GREEN2= 6 

Buzzer=24        

Red_sig1=14
Green_sig1=15 
Red_sig2=18
Green_sig2=23


GPIO.setup(Buzzer, GPIO.OUT)

GPIO.setup(Red_sig1, GPIO.OUT)
GPIO.setup(Green_sig1, GPIO.OUT)
GPIO.setup(Red_sig2, GPIO.OUT)
GPIO.setup(Green_sig2, GPIO.OUT)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

GPIO.setup(RED1, GPIO.OUT)
GPIO.setup(GREEN1, GPIO.OUT)
GPIO.setup(RED2, GPIO.OUT)
GPIO.setup(GREEN2, GPIO.OUT)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.output(RED1, False)
GPIO.output(GREEN1, False)
GPIO.output(RED2, False)
GPIO.output(GREEN2, False)

GPIO.output(Red_sig1, False)
GPIO.output(Green_sig1, False)
GPIO.output(Red_sig2, False)
GPIO.output(Green_sig2, False)

GPIO.output(IN1, False)
GPIO.output(IN2, False)
GPIO.output(IN3, False)
GPIO.output(IN4, False)


GPIO.output(Buzzer, False)

###lcd #####################

LCD_RS = 8
LCD_E  = 7
LCD_D4 = 22
LCD_D5 = 10
LCD_D6 = 9
LCD_D7 = 11


# Define some device constants
LCD_WIDTH = 20    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x90 # LCD RAM address for the 2nd line
LCD_LINE_4 = 0xD0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

####lcd end here###########S
gpio.setwarnings(False)
gpio.setmode(gpio.BCM)

#gpio.setup(22, gpio.IN, pull_up_down=gpio.PUD_UP)#Button to gpio12
      # Use BCM gpio numbers
gpio.setup(LCD_E, gpio.OUT)  # E
gpio.setup(LCD_RS, gpio.OUT) # RS
gpio.setup(LCD_D4, gpio.OUT) # DB4
gpio.setup(LCD_D5, gpio.OUT) # DB5
gpio.setup(LCD_D6, gpio.OUT) # DB6
gpio.setup(LCD_D7, gpio.OUT) # DB7

####################LCD#######################
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  gpio.output(LCD_RS, mode) # RS

  # High bits
  gpio.output(LCD_D4, False)
  gpio.output(LCD_D5, False)
  gpio.output(LCD_D6, False)
  gpio.output(LCD_D7, False)
  if bits&0x10==0x10:
    gpio.output(LCD_D4, True)
  if bits&0x20==0x20:
    gpio.output(LCD_D5, True)
  if bits&0x40==0x40:
    gpio.output(LCD_D6, True)
  if bits&0x80==0x80:
    gpio.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  gpio.output(LCD_D4, False)
  gpio.output(LCD_D5, False)
  gpio.output(LCD_D6, False)
  gpio.output(LCD_D7, False)
  if bits&0x01==0x01:
    gpio.output(LCD_D4, True)
  if bits&0x02==0x02:
    gpio.output(LCD_D5, True)
  if bits&0x04==0x04:
    gpio.output(LCD_D6, True)
  if bits&0x08==0x08:
    gpio.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  gpio.output(LCD_E, True)
  time.sleep(E_PULSE)
  gpio.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display
  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

lcd_init()
lcd_byte(0x01, LCD_CMD)
lcd_string(" MOVABLE ROAD ",LCD_LINE_1)
lcd_string("   DIVIDER   ",LCD_LINE_2)
time.sleep(2)

import socket
TCP_IP = '192.168.43.155'
TCP_PORT = 8081
BUFFER_SIZE = 20
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn, addr = s.accept()
print ('Connection address:', addr)
ck=1

def ALL_ZERO():

    GPIO.output(RED1, False)
    GPIO.output(GREEN1, False)
    GPIO.output(RED2, False)
    GPIO.output(GREEN2, False)

def ALL_HIGH():

    GPIO.output(RED1, True)
    GPIO.output(GREEN1, True)
    GPIO.output(RED2, False)
    GPIO.output(GREEN2, False)
    
def MOVE_DIVIDER_LEFT():
    print("Ambulance arriving Right")
    lcd_byte(0x01, LCD_CMD)
    lcd_string("DIVIDER MOVING",LCD_LINE_1)
    lcd_string("Left..",LCD_LINE_2)
    time.sleep(1)
    #for divider movement
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    time.sleep(2)
    
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

def MOVE_DIVIDER_RIGHT():
    print("Ambulance on Left")
    lcd_byte(0x01, LCD_CMD)
    lcd_string("DIVIDER MOVING",LCD_LINE_1)
    lcd_string("Right...",LCD_LINE_2)
    time.sleep(1)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)
    
    
    time.sleep(2)
    
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)



while ck==1:
    data = conn.recv(BUFFER_SIZE)
    data=data.decode('UTF-8','ignore')

    print ("received data:", str(data))
    if str(data) == 'A':
##    if(road_flag == 1):
##        print("Amblancce in ROAD1 already ROAD CLEARED FOR TRAFFIC")
##
##    else:
            print("Amblancce in left")
            lcd_byte(0x01, LCD_CMD)
            lcd_string("AMBULANCE IN left",LCD_LINE_1)
            time.sleep(1)
            GPIO.output(Red_sig1, True)
            GPIO.output(Green_sig1, False)
                
            GPIO.output(Red_sig2, False)
            GPIO.output(Green_sig2,True)
            time.sleep(1)
            ALL_ZERO()
            time.sleep(1)
            GPIO.output(RED1,False)
            GPIO.output(GREEN2,False)
            GPIO.output(RED2,True)
            GPIO.output(GREEN1,True)            
            MOVE_DIVIDER_RIGHT();
            time.sleep(1)
            ALL_ZERO()
            time.sleep(1)
            ALL_HIGH()
            time.sleep(1)
##            break

        
    if str(data) == 'B':
##         if(road_flag == 2):
##             print("Amblancce in ROAD 2 already ROAD CLEARED FOR TRAFFIC")
##             
##         else:
             print("Amblancce in ROAD 2")
             GPIO.output(Red_sig1, False)
             GPIO.output(Green_sig1,True)
             lcd_byte(0x01, LCD_CMD)
             lcd_string("AMBULANCE IN ROAD2 ",LCD_LINE_1)
             time.sleep(1)              
             GPIO.output(Red_sig2, True)
             GPIO.output(Green_sig2, False)
             time.sleep(1)
             ALL_ZERO()
             time.sleep(1)
            #RED2 is right side signal
             GPIO.output(RED2,False)
             GPIO.output(GREEN1,False)
             GPIO.output(RED1,True)
             GPIO.output(GREEN2,True)
             MOVE_DIVIDER_LEFT();
             time.sleep(1)
             ALL_ZERO()
             time.sleep(1)
             ALL_HIGH()
             time.sleep(1)
##             break
    GPIO.output(RED1, False)
    GPIO.output(GREEN1, False)
    GPIO.output(RED2, False)
    GPIO.output(GREEN2, False)
                                         
        
conn.close()
