import smbus2
import RPi.GPIO as GPIO
import time

max7320_address = 0x58
max7321_address = 0x68
max7322_address = 0x6c

adsd3500_address = 0x38

ENABLE_VAUX_BITS = 0x01
ENABLE_VSYS_BITS = 0x02
ENABLE_FSYNC_BITS = 0x80

ENABLE_C0V8_BITS = 0x80
ENABLE_C1V8_BITS = 0x40
ENABLE_DS2_BITS = 0x20

BRD_RST = 21
FSYNC = 20

class max7320:
    def __init__(self,i2c):
        if i2c is None:
            raise ValueError('I2C Object required.')
        self.i2c = i2c
        self.address = max7320_address
        self.max7320_init()

    def max7320_init(self):
        self.max7320_clear()

    def max7320_clear(self):
        self.max7320_write(0x00)
        print(f"Clear max7320 power tree.")

    def max7320_write(self,value):
        self.i2c.write_byte(self.address,value,force=None)

    def max7320_read(self):
        data = self.i2c.read_byte(self.address,force=None)
        print(f"Line 32: Data: {hex(data)}")
        return data

    def Enable_CarrierBoard_0V8_1V8(self):

        data = self.max7320_read()
        data = data | ENABLE_C1V8_BITS
        self.max7320_write(data)
        print(f"C1V8 Power Enable.")
        time.sleep(0.1)

        data = self.max7320_read()
        data = data | ENABLE_C0V8_BITS
        self.max7320_write(data)
        print(f"C0V8 Power Enable.")
        time.sleep(0.1)

        data = self.max7320_read()
        data = data | ENABLE_DS2_BITS
        self.max7320_write(data)
        print(f"DS2 Enable.")
        self.max7320_read()
        time.sleep(0.1)

class max7322:
    def __init__(self,i2c):
        if i2c is None:
            raise ValueError('I2C Object required.')
        self.i2c = i2c
        self.address = max7322_address
        self.max7322_init()

    def max7322_init(self):
        self.max7322_clear()

    def max7322_clear(self):
        self.max7322_write(0x00)
        print(f"Clear max7322 power tree.")

    def max7322_write(self,value):
        self.i2c.write_byte(self.address,value,force=None)

    def max7322_read(self):
        data = self.i2c.read_byte(self.address,force=None)
        print(f"Line 28: Data: {hex(data)}")
        return data

    def max7322_boot(self):
        data = self.max7322_read()
        data = data | ENABLE_VSYS_BITS
        self.max7322_write(data)
        print(f"VSYS Power Enable.")
        time.sleep(0.1)
        data = self.max7322_read()
        data = data | ENABLE_VAUX_BITS
        self.max7322_write(data)
        print(f"VAUX Power Enable.")
        time.sleep(0.1)
        data = self.max7322_read()
        data = data | ENABLE_FSYNC_BITS
        self.max7322_write(data)
        print(f"FSYNC Enable.")
        time.sleep(0.1)
        self.max7322_read()

class max7321:
    def __init__(self,i2c):
        if i2c is None:
            raise ValueError('I2C Object required.')
        self.i2c = i2c
        self.address = max7321_address
        self.max7321_init()

    def max7321_init(self):
        self.max7321_clear()

    def max7321_clear(self):
        self.max7321_write(0x00)
        print(f"Clear max7321 power tree.")

    def max7321_write(self,value):
        self.i2c.write_byte(self.address,value,force=None)

    def max7321_read(self):
        data = self.i2c.read_byte(self.address,force=None)
        print(f"Line 28: Data: {hex(data)}")
        return data
    
    def Boot_Mode(self):
        self.max7321_write(0x80)
        data = self.max7321_read()
        print(f"BOOT Mode End:{hex(data)}.")

if __name__=='__main__':

    i2c = smbus2.SMBus(10)
    #step 1 enable carrier board
    max7322 = max7322(i2c)
    max7322.max7322_boot()
    
    #STEP 2 enable reset adsd3500
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BRD_RST, GPIO.OUT)
    #GPIO.setup(FSYNC, GPIO.IN)
    GPIO.output(BRD_RST, GPIO.LOW)
    #STEP 3 Config OC0-0C6

    max7321 = max7321(i2c)
    max7321.Boot_Mode()

    max7320 = max7320(i2c)
    max7320.Enable_CarrierBoard_0V8_1V8()

    GPIO.output(BRD_RST, GPIO.HIGH)