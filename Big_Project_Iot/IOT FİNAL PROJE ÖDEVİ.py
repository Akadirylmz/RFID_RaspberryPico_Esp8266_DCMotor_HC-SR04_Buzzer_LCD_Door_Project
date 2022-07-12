import uos
import machine
import utime
import time
from machine import Pin
from machine import I2C, SPI
from mfrc522 import MFRC522
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd

myHOST = '184.106.153.149'
myPORT = '80'
myAPI = 'yourApi'

print()
print("Machine: \t" + uos.uname()[4])
print("MicroPython: \t" + uos.uname()[3])

I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16
i2c = I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)


trig = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN, Pin.PULL_DOWN)
buzzer= Pin(16, Pin.OUT)
in1 = Pin(26, Pin.OUT)
in2 = Pin(27, Pin.OUT)
buzzer.value(1)
mavi = Pin(13, Pin.OUT)
kırmızı = Pin(12, Pin.OUT)
sck = Pin(6, Pin.OUT)
mosi = Pin(7, Pin.OUT)
miso = Pin(4, Pin.OUT)
sda = Pin(5, Pin.OUT)
rst = Pin(18, Pin.OUT)
spi = SPI(0, baudrate=100000, polarity=0, phase=0, sck=sck, mosi=mosi, miso=miso)
card1 = "0x93f7eb0c"

uart0 = machine.UART(0, baudrate=115200)
print(uart0)

def Rx_ESP_Data():
    recv=bytes()
    while uart0.any()>0:
        recv+=uart0.read(1)
    res=recv.decode('utf-8')
    return res
def Connect_WiFi(cmd, uart=uart0, timeout=3000):
    print("CMD: " + cmd)
    uart.write(cmd)
    utime.sleep(7.0)
    Wait_ESP_Rsp(uart, timeout)
    print()

def Send_AT_Cmd(cmd, uart=uart0, timeout=3000):
    print("CMD: " + cmd)
    uart.write(cmd)
    Wait_ESP_Rsp(uart, timeout)
    print()
    
def Wait_ESP_Rsp(uart=uart0, timeout=3000):
    prvMills = utime.ticks_ms()
    resp = b""
    while (utime.ticks_ms()-prvMills)<timeout:
        if uart.any():
            resp = b"".join([resp, uart.read(1)])
    print("resp:")
    try:
        print(resp.decode())
    except UnicodeError:
        print(resp)
    
Send_AT_Cmd('AT\r\n')          #Test AT startup
Send_AT_Cmd('AT+GMR\r\n')      #Check version information
Send_AT_Cmd('AT+CIPSERVER=0\r\n')      #Check version information
Send_AT_Cmd('AT+RST\r\n')      #Check version information
Send_AT_Cmd('AT+RESTORE\r\n')  #Restore Factory Default Settings
Send_AT_Cmd('AT+CWMODE?\r\n')  #Query the Wi-Fi mode
Send_AT_Cmd('AT+CWMODE=1\r\n') #Set the Wi-Fi mode = Station mode
Send_AT_Cmd('AT+CWMODE?\r\n')  #Query the Wi-Fi mode again
Connect_WiFi('AT+CWJAP="WİFİ_NAME_WRİTE","WRİTE_TO_WİFİ_PASSWORD"\r\n', timeout=5000) #Connect to AP
Send_AT_Cmd('AT+CIFSR\r\n',timeout=5000)    #Obtain the Local IP Address
Send_AT_Cmd('AT+CIPMUX=1\r\n')    #Obtain the Local IP Address
utime.sleep(1.0)
print ('Starting connection to ESP8266...')

while True:
    
    trig.value(0)
    time.sleep(0.1)
    trig.value(1)
    time.sleep_us(2)
    trig.value(0)
    while echo.value()==0:
         pulse_start = time.ticks_us()
    while echo.value()==1:
         pulse_end = time.ticks_us()
    pulse_duration = pulse_end - pulse_start
    dist = pulse_duration * 17165 / 1000000
    distance = str(pulse_duration * 17165 / 1000000)
    
    
    rdr = MFRC522(spi, sda, rst)
    (stat, tag_type) = rdr.request(rdr.REQIDL)
    if stat == rdr.OK:
        (stat, raw_uid) = rdr.anticoll()
        if stat == rdr.OK:
            uid = ("0x%02x%02x%02x%02x" % (raw_uid[0], raw_uid[1], raw_uid[2], raw_uid[3]))
            
            if uid == card1:
                print("Kart Algılandı!")
                buzzer.value(0)
                mavi.value(1)
                in1.value(1)
                in2.value(0)
                time.sleep(3)
                buzzer.value(1)
                in1.value(0)
                in2.value(0)
                mavi.value(0)
                time.sleep(1)
                lcd.move_to(0,0)
                lcd.putstr("UZAKLIK: "+str("{:.0f}".format(dist))+"cm    ")
                lcd.move_to(0,1)
                lcd.putstr("Kart Algilandi!")
                time.sleep(0.1)
         
               
                
            else:
                print("Geçersiz Kart!!!")
                buzzer.value(0)
                kırmızı.value(1)
                time.sleep(1.5)
                buzzer.value(3)
                kırmızı.value(0)
                time.sleep(1)
                lcd.move_to(0,0)
                lcd.putstr("Gecersiz Kart!!!")
                lcd.move_to(0,1)
                lcd.putstr("                ")
                time.sleep(0.1)
            
                
    print ('UZAKLIK:',dist,'cm')
    
    print ('!About to send data to thingspeak')
    sendData = 'GET /update?api_key='+ myAPI +'&field1='+distance
    Send_AT_Cmd('AT+CIPSTART=0,\"TCP\",\"'+ myHOST +'\",'+ myPORT+'\r\n')
    utime.sleep(1.0)
    Send_AT_Cmd('AT+CIPSEND=0,' +str(len(sendData)+4) +'\r\n')
    utime.sleep(1.0)
    Send_AT_Cmd(sendData +'\r\n')
    utime.sleep(4.0)
    Send_AT_Cmd('AT+CIPCLOSE=0'+'\r\n') # once file sent, close connection
    utime.sleep(4.0)
    print ('Data send to thing speak')