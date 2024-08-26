#*************** BELOW ARE PARAMETERS THAT AFFECT TRIGGER SENSITIVTY **********************************************************************
DO_MOTION_DETECT = True     # if true the device will ignore the timed IMAGE_HOUR_X settings
CAMERA_SLEEP_INTERVAL = 1       # Number of seconds the camera goes into light sleep.
MIN_VALID_BLOB_SIZE = 30        # A moving blob larger than this many pixels is considered bogus and is not saved.
MAX_VALID_BLOB_SIZE = 450       # A moving blob smaller than this many pixels is considered bogus and is not saved. was 1000 April 15
TOO_MANY_DIFFERENT_PIXELS = 500 # If more than this many pixels changed then consider it a bogus shift in background. was 500 April 15
TOO_MANY_MOVING_BLOBS = 100       # If more than this many moving blobs consider it a bogus shift in background.
THRESHOLD_FOR_BLOBS = 20         # Amount of luminance distinguishing pixels that changed from pixels that didn't change.
DETECTION_DIFFS = 40           # Total of background subtraction deltas that triggers a detection.
SAVE_DETECTIONS = True         # Set to True if want to save detected images, False if just playing with code in the lab.
#*********************************************************************************************************************
DO_LORAWAN = False
DO_WEATHER_SENSORS = True
USE_IR_FLASH = True
FLASH_ON_THRESHOLD = 12
FLASH_OFF_THRESHOLD = 22
FLASH_STATUS = False
#*********************************************************************************************************************
FWver = '01.06'
MAX_FILES_PER_FOLDER = 1500 #Feb 2024: Works at 1000, fails at 2000.
FOLDER_LETTER_MAX = 90
FOLDER_LETTER_INIT = 65
LRW_TIMING_OFFSET = 15
PRINT_DEBUGS = True             # Sends debug info to the Serial Terminal window at the bottom of OpenMV UI. Normally off (False) for field deployments.
UART_DEBUGS = True
#2_6FF9  5   (SoCal)
#3_7FE5  10  (SoCal)
#5_327E  15  (SoCal)
#1_9877  5   (CR)
#8_2F7D  10  (CR)
#9_021D  15  (CR)
GPS_MIN = 3  #GMT Time to take a GPS
IMAGE_MIN = 666  #GMT Time to take a GPS
GPS_HOUR_A = 2  #GMT Time to take a GPS
GPS_HOUR_B = 12  #GMT Time to take a second GPS (set to -1 if no second GPS / day)
#*************** END OF CONFIGS ******************************************************
#*************** VERSION HISTORY *****************************************************
# Added LRW pwr cycle
# Reversed longitude sign
INITVAL = 85
DO_MINUTE_FUNCTION = True  #does various tasks once per minute like saving weather sensors.

import sensor, image, time, pyb, utime, omv, uos, ustruct, machine, stm

SAM_USE_AUTO_LIDAR = 1
SAM_USE_FIXED_MIN = 20  #byte
SAM_USE_FIXED_MAX = 50  #short
SAM_MAX_DAILY_MOTION_PICS = 200  #short
LRW_FORMAT = 2 #(2=image slice)
WATCHDOG_ADDRESS = const(0x58004800)
WATCHDOG_KEY = const(0xAAAA)
LCD_DELAY = 0
packetToLRW=[0]*55

def filterSwitchNight():
    FILTER_SW_B.init(FILTER_SW_B.IN, FILTER_SW_B.PULL_NONE)     #PD4 = FILTER_SW_B
    FILTER_SW_A.init(FILTER_SW_A.OUT_PP, FILTER_SW_A.PULL_NONE) #PG14 = FILTER_SW_A
    FILTER_SW_A.value(0)
    pyb.delay(40)
    FILTER_SW_A.init(FILTER_SW_A.IN, FILTER_SW_A.PULL_NONE)     #PG14 = FILTER_SW_A

def filterSwitchDay():
    FILTER_SW_A.init(FILTER_SW_A.IN, FILTER_SW_A.PULL_NONE)     #PG14 = FILTER_SW_A
    FILTER_SW_B.init(FILTER_SW_B.OUT_PP, FILTER_SW_B.PULL_NONE) #PG14 = FILTER_SW_B
    FILTER_SW_B.value(0)
    pyb.delay(40)
    FILTER_SW_B.init(FILTER_SW_B.IN, FILTER_SW_B.PULL_NONE)     #PD4 = FILTER_SW_B

def camFlash(option):
    if(option==0):
        IR_FLASH_A.init(IR_FLASH_A.IN, IR_FLASH_A.PULL_NONE)
        IR_FLASH_B.init(IR_FLASH_B.IN, IR_FLASH_B.PULL_NONE)
    elif(option==1):
        IR_FLASH_A.init(IR_FLASH_A.OUT_PP, IR_FLASH_A.PULL_NONE)
        IR_FLASH_A.value(0)
    elif(option==2):
        IR_FLASH_B.init(IR_FLASH_B.OUT_PP, IR_FLASH_B.PULL_NONE)
        IR_FLASH_B.value(0)
    elif(option==3):
        IR_FLASH_A.init(IR_FLASH_A.OUT_PP, IR_FLASH_A.PULL_NONE)
        IR_FLASH_A.value(0)
        IR_FLASH_B.init(IR_FLASH_B.OUT_PP, IR_FLASH_B.PULL_NONE)
        IR_FLASH_B.value(0)

def LCDinit():  #SEE LCDoff() for LCD BUG.
#    pyb.delay(350)   #JUNE 2024: NEED MORE THAN 285ms BETWEEN LCDoff() and LCDinit(), otherwise no display of text.
    LCD_SI.low()   #A0=LCD SI, serial data
    LCD_PWRC.low()  #G9=LCD PWRC, low=OFF
    LCD_RS.low()   #E3=LCD Register Select, 0=instruction, 1=data
    LCD_SCL.low()  #D3=LCD serial clock
    pyb.delay(300)  #FAILS at 100. Okay at 200.
    LCD_SI.high()   #A0=LCD SI, serial data
    LCD_PWRC.low()  #G9=LCD PWRC, low=OFF
    LCD_RS.high()   #E3=LCD Register Select, 0=instruction, 1=data
    LCD_SCL.high()  #D3=LCD serial clock
    pyb.delay(10)
    LCD_PWRC.high()  #G9=LCD PWRC, low=OFF
    pyb.delay(200)   #FAILS at 10. Okay at 20.
    LCDwriteCommandByte(0x30) #WakeUp
    pyb.delay(2)
    LCDwriteCommandByte(0x30) #WakeUp
    LCDwriteCommandByte(0x30) #WakeUp
    LCDwriteCommandByte(0x39) #Function Set
    LCDwriteCommandByte(0x14) #Internal Oscillator Frequency
    LCDwriteCommandByte(0x56) #Power Control
    LCDwriteCommandByte(0x6D) #Follower Control
    LCDwriteCommandByte(0x70) #Contrast
    LCDwriteCommandByte(0x0C) #Display On
    LCDwriteCommandByte(0x06) #Entry Mode
    LCDwriteCommandByte(0x01) #Clear
    pyb.delay(50)

def LCDoff():
    LCD_SI.low()    #A0=LCD SI, serial data
    LCD_SCL.low()   #D3=LCD serial clock
    LCD_RS.low()   #E3=LCD Register Select, 0=instruction, 1=data
    LCD_PWRC.low() #G13=LCD PWRC, low=ON

def LCDshort(a):
    numstr = str(a)
    numlen = len(numstr)
    barray = numstr.encode('ascii', 'UTF-8')
    for iters in range (numlen,0,-1):
        LCDwriteDataByte((barray[numlen-iters]))

def LCDwriteLine(lineToWrite, topBottom):
    if (topBottom):
        LCDwriteCommandByte(0x80)
    else:
        LCDwriteCommandByte(0xC0)
    for iters in range (0, 16):
        LCDwriteDataByte(lineToWrite[iters])

def LCDwriteDataByte(byteToWrite):
    LCD_RS.high()       #E3=LCD Register Select, 0=instruction, 1=data
    for iters in range (1,9):
        if ((byteToWrite & 0x80)==0x80):
            LCD_SI.high()
        else:
            LCD_SI.low()
        byteToWrite=(byteToWrite<<1)
        LCD_SCL.low()   #D3=LCD serial clock
        LCD_SCL.high()  #D3=LCD serial clock
        LCD_SCL.low()   #D3=LCD serial clock
    pyb.delay(1)

def LCDwriteCommandByte(cbyteToWrite):
    LCD_RS.low()        #E3=LCD Register Select, 0=instruction, 1=data
    for iters in range (1,9):
        if ((cbyteToWrite & 0x80)==0x80):
            LCD_SI.high()
        else:
            LCD_SI.low()
        cbyteToWrite=(cbyteToWrite<<1)
        LCD_SCL.low()   #D3=LCD serial clock
        LCD_SCL.high()  #D3=LCD serial clock
        LCD_SCL.low()   #D3=LCD serial clock
    LCD_RS.high()       #E3=LCD Register Select, 0=instruction, 1=data
    pyb.delay(1)

def write_sda_EE (x):
    if(x):
        eepromSDA_INpull()
    else:
        eepromSDA_OUTlow()
def i2c_write_byte_EE(byte, stretch):
    for bitnum in range (0, 8):
        write_sda_EE((byte & 0x80) != 0)
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
        eepromSCL_INpull()
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
        eepromSCL_OUTlow()
        byte <<= 1
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
    eepromSDA_INpull()
    eepromSCL_INpull()  #goes high for the 9th clock
    pyb.udelay(0)        #delay_us(DELAY_L); was one usec with Atmel
    pyb.udelay(0)        #delay_us(4); was FOUR usec with Atmel
    #Check for ack
    if ((stretch==0) and EEPROM_SDA.value()):    #(HAL_GPIO_ReadPin(GPIOB, 1<<11)):
#        print("NO ACK")
        twi_stop_cond_EE()
#        if (PRINT_DEBUGS):
#            print("NO ACK")
        return 0        #If no ACK from slave, return 0.
    pyb.udelay(1)        #delay_us(DELAY_L); was one usec with Atmel
    eepromSCL_OUTlow()  #end of byte with acknowledgment.
    eepromSDA_OUTlow()
    if(stretch):
        pyb.udelay(EEDelay)
    if (stretch and EEPROM_SDA.value()):    #(HAL_GPIO_ReadPin(GPIOB, 1<<11)):
#        print("STRETCH NO ACK")
        twi_stop_cond_EE()
#    elif stretch:
#        print("GOT STRETCH ACK")
    return 1
def i2c_write_byte_EE_WRITE(byte, stretch):
    for bitnum in range (0, 8):
        write_sda_EE((byte & 0x80) != 0)
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
        eepromSCL_INpull()
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
        eepromSCL_OUTlow()
        byte <<= 1
        pyb.udelay(EEDelay)	#delay_us(DELAY_L); was one usec with Atmel
    eepromSDA_INpull()
    eepromSCL_INpull()  #goes high for the 9th clock
    pyb.udelay(0)        #delay_us(DELAY_L); was one usec with Atmel
    pyb.udelay(0)        #delay_us(4); was FOUR usec with Atmel
    eepromSCL_OUTlow()  #end of byte with acknowledgment.
    eepromSDA_OUTlow()
    return 1

def eepromSDA_INpull():
    EEPROM_SDA.init(EEPROM_SDA.IN, EEPROM_SDA.PULL_UP)
def eepromSCL_INpull():
    EEPROM_SCL.init(EEPROM_SCL.IN, EEPROM_SCL.PULL_UP)
def eepromSDA_OUTlow():
    EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
    EEPROM_SDA.value(0)
def eepromSCL_OUTlow():
    EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
    EEPROM_SCL.value(0)
def eepromSDA_OUT():
    EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
def eepromSCL_OUT():
    EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
def twi_start_cond_EE_WRITE():
    eepromSCL_INpull()
    eepromSDA_INpull()
    pyb.udelay(EEDelay)
    eepromSDA_OUTlow()
    pyb.udelay(EEDelay)
    eepromSCL_OUTlow()
    pyb.udelay(EEDelay)
def twi_start_cond_EE():
    eepromSCL_INpull()
    eepromSDA_OUTlow()
    pyb.udelay(EEDelay)
    eepromSCL_OUTlow()
    pyb.udelay(EEDelay)
def twi_stop_cond_EE():
    eepromSDA_OUT()
    pyb.udelay(EEDelay)	#delay_us(DELAY_L); was usec in Atmel
    eepromSCL_INpull()
    pyb.udelay(EEDelay)	#delay_us(DELAY_L); was usec in Atmel
def twi_stop_cond_EE_WRITE():
    eepromSCL_INpull()
    pyb.udelay(EEDelay)	#delay_us(DELAY_L); was usec in Atmel
    pyb.udelay(EEDelay)	#delay_us(DELAY_L); was usec in Atmel
    eepromSDA_OUTlow()
    pyb.udelay(EEDelay)	#delay_us(DELAY_L); was usec in Atmel
    eepromSDA_INpull()
def send_slave_Address_EE(readBit, slaveAddress):
    i2c_write_byte_EE(slaveAddress | readBit, 0)

def read_bytes_EE(data, numBytes, startAddress, slaveAddress):
    index=0
    twi_start_cond_EE()
    send_slave_Address_EE(0, slaveAddress)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(0xFF,0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(startAddress,0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    eepromSCL_INpull()
    pyb.udelay(EEDelay)
    eepromSDA_INpull()
    pyb.udelay(50)
    twi_start_cond_EE()
    i2c_write_byte_EE((0xA3),0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    for index in range (0, numBytes):
        i2c_read_byte_EE(data, numBytes, index)
        eepromSCL_OUTlow()
        eepromSDA_OUT()
        pyb.udelay(EEDelay)
    i2c_read_byte_EE(dataDum, 1, 0)  #Last byte written is 1 to signal no ACK
    twi_stop_cond_EE()

def i2c_read_byte_EE(data, numBytes, index):
    byte = 0
    bitnum = 0
    eepromSDA_INpull()
    for bitnum in range (0,8):
       eepromSCL_INpull()
       pyb.udelay(EEDelay)     #delay_us(DELAY_L); was one usec with Atmel
       if EEPROM_SDA.value():
          byte |= (1 << (7- bitnum))
       eepromSCL_OUTlow()
       pyb.udelay(EEDelay)     #delay_us(DELAY_L); was one usec with Atmel
    data[index] = byte
    if(index < (numBytes-1)):  #if not the last byte, ACK the slave
       eepromSDA_OUTlow()	#ACK
       eepromSCL_INpull()  #SCL high for 9th clock
       pyb.udelay(EEDelay)        #delay_us(DELAY_L); was one usec with Atmel
       eepromSCL_OUTlow()  #SCL low after 9th clock
       pyb.udelay(EEDelay)        #delay_us(DELAY_L); was one usec with Atmel
       eepromSDA_INpull()  #release SDA after ACK
    else: #send NACK on the last byte
       eepromSDA_INpull()  #NACK
       eepromSCL_INpull()  #SCL high for the 9th clock
       pyb.udelay(EEDelay)        #delay_us(DELAY_L); was one usec with Atmel
       eepromSCL_OUTlow()  #SCL low after 9th clock
       pyb.udelay(EEDelay)        #delay_us(DELAY_L); was one usec with Atmel
       eepromSCL_INpull()  #goes high for the 9th clock
       pyb.udelay(0)        #delay_us(DELAY_L); was one usec with Atmel
       while(EEPROM_SCL.value()==0):
            pass

def eepromWriteAsciiLatLong(asciiLatLong):
    latLongArray = bytearray(asciiLatLong)
    twi_start_cond_EE_WRITE()
    send_slave_Address_EE(0, 0xA2)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(0xFF,0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(0xE4,0)   #E4
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    for i in range (0, 19):  #for each element
        i2c_write_byte_EE((latLongArray[i]),0)
        eepromSCL_OUTlow()
        eepromSDA_OUT()
        pyb.udelay(EEDelay)
    twi_stop_cond_EE_WRITE()
    pyb.delay(6)    #needs this

def eepromWriteCompressedLatLong(compressed, hiLo):
    twi_start_cond_EE_WRITE()
    send_slave_Address_EE(0, 0xA2)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(0xFF,0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE(hiLo,0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE((compressed >> 24),0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE((compressed >> 16),0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE((compressed >> 8),0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    i2c_write_byte_EE((compressed & 0x000000FF),0)
    eepromSCL_OUTlow()
    eepromSDA_OUT()
    pyb.udelay(EEDelay)
    twi_stop_cond_EE_WRITE()
    pyb.delay(6)    #needs this!

@micropython.asm_thumb  #example uses the LED
def setBitRegisterTEMPLATE() -> uint:  #these asm functions return r0
    movwt (r3, 0x58020814)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register
    mov (r1, 1)     #r1 contains the mask
    mov (r2, 1)     #r2 this is how many times the mask will be shifted
    lsl (r1, r2)    #shift the r1 mask by r2 bits
    orr (r0, r1)    #set the bit(s) in r0 per the mask in r1
    str (r0, [r3, 0])   #store r0 into location r3

@micropython.asm_thumb  #example uses the LED
def setBitRegisterWKUPEPR() -> uint:  #these asm functions return r0
    movwt (r3, 0x58024828)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register
    mov (r1, 3)     #r1 contains the mask
#    mov (r2, 1)     #r2 this is how many times the mask will be shifted
    lsl (r1, r2)    #shift the r1 mask by r2 bits
    orr (r0, r1)    #set the bit(s) in r0 per the mask in r1
    str (r0, [r3, 0])   #store r0 into location r3

@micropython.asm_thumb  #example uses the LED
def setBitRegisterEXTIIMR2() -> uint:  #these asm functions return r0
    movwt (r3, 0x58000090)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register
    mov (r1, 3)     #r1 contains the mask
    mov (r2, 23)     #r2 this is how many times the mask will be shifted
    lsl (r1, r2)    #shift the r1 mask by r2 bits
    orr (r0, r1)    #set the bit(s) in r0 per the mask in r1
    str (r0, [r3, 0])   #store r0 into location r3

@micropython.asm_thumb  #example uses the LED
def setBitRegisterEXTIEMR2() -> uint:  #these asm functions return r0
    movwt (r3, 0x58000094)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register
    mov (r1, 3)     #r1 contains the mask
    mov (r2, 23)     #r2 this is how many times the mask will be shifted
    lsl (r1, r2)    #shift the r1 mask by r2 bits
    orr (r0, r1)    #set the bit(s) in r0 per the mask in r1
    str (r0, [r3, 0])   #store r0 into location r3

@micropython.asm_thumb  #example uses the LED
def clearBitRegisterTEMPLATE() -> uint:  #these asm functions return r0
    movwt (r3, 0x58020814)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register
    mov (r1, 1)     #r1 contains the mask
    mov (r2, 1)     #r2 this is how many times the mask will be shifted
    lsl (r1, r2)    #shift the r1 mask by r2 bits
    mvn (r1, r1)    #1's complement the mask
    and_ (r0, r1)    #clear the bit(s) in r0 per the mask in r1
    str (r0, [r3, 0])   #store r0 into location r3

@micropython.asm_thumb
def getRegisterTEMPLATE() -> uint:  #these asm functions return r0
    movwt (r3, 0x58020814)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

@micropython.asm_thumb
def getRegisterEXTI_IMR2() -> uint:  #these asm functions return r0
    movwt (r3, 0x58000090)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

@micropython.asm_thumb
def getRegisterEXTI_EMR2() -> uint:  #these asm functions return r0
    movwt (r3, 0x58000094)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

@micropython.asm_thumb
def getRegisterPWR_WKUPFR() -> uint:  #these asm functions return r0
    movwt (r3, 0x58024824)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

@micropython.asm_thumb
def getRegisterPWR_WKUPEPR() -> uint:  #these asm functions return r0
    movwt (r3, 0x58024828)  #GPIOC = 0x58020800. GPIOC output register = 0x58020814
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

@micropython.asm_thumb
def getRegisterRTC_ISR() -> uint:  #these asm functions return r0
    movwt (r3, 0x5800400C)  #RTC = 0x5800400C. RTC->ISR register = 0x58004
    ldr (r0, [r3, 0])   #r0 now has the contents of the register

def calcCRC(cbuff, LEN): #This matches online calculator for CRC-16/AUG-CCITT Poly: 0x1021 Init: 0xFFFF. crccalc.com
    X = 0xFFFF
    Y = 0x0080
    Z = 0
    cbuff = bytearray(cbuff)
    for i in range (0, LEN):  #for each element
        Y = 0x0080
        for j in range (0,8):
            Z = X
            Z &= 0xFFFF
            X <<= 1
            X &= 0xFFFF
            if((Y & cbuff[i]) != 0): #if cbuff[i] is negative
                X += 1
                X &= 0xFFFF
            Y >>= 1
            if ((Z & 0x8000) != 0): #if Z is negative
                X ^= 0x1021        #end of each element
                X &= 0xFFFF
    for i in range (0, 16):
        if ((X & 0x8000) != 0):
            X <<= 1
            X &= 0xFFFF
            X ^= 0x1021
            X &= 0xFFFF
        else:
            X <<= 1
            X &= 0xFFFF
    return X

def displayMenu():

    eeReadData=[0]*21
    read_bytes_EE(eeReadData,20,0xE4,0xA2)

    gotTemperature=0
    humid=0
    gotBarom=0
    als=0
    LCDinit()
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    LCDwriteCommandByte(0x80)   #80=Top Line
    LCDwriteDataByte(ord('F'))
    LCDwriteDataByte(ord('I'))
    LCDwriteDataByte(ord('E'))
    LCDwriteDataByte(ord('L'))
    LCDwriteDataByte(ord('D'))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('D'))
    LCDwriteDataByte(ord('A'))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord('A'))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord('E'))
    LCDwriteDataByte(ord('C'))
    LCDwriteDataByte(ord('H'))
    LCDwriteCommandByte(0xC0)   #C0=Bottom Line
    uniqueID = omv.board_id()
    bID = bytearray()  #declares bID as a bytearray
    bID.extend(uniqueID.encode())
    crcrc = calcCRC(bID,24)
    crcStr = (hex(crcrc)).upper()[2:6]
    while (len(crcStr)<4):  #if crc is less than 4 chars pad with leading zeros
        crcStr = '0' + crcStr
    LCDwriteDataByte(ord(crcStr[0]))
    LCDwriteDataByte(ord(crcStr[1]))
    LCDwriteDataByte(ord(crcStr[2]))
    LCDwriteDataByte(ord(crcStr[3]))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('v'))
    LCDwriteDataByte(ord(FWver[0]))
    LCDwriteDataByte(ord(FWver[1]))
    LCDwriteDataByte(ord(FWver[2]))
    LCDwriteDataByte(ord(FWver[3]))
    LCDwriteDataByte(ord(FWver[4]))
    pyb.delay(1500)
#*********** DATE TIME ***********************
    yy,MM,dd,ww,hh,mm,ss,uu = rtc.datetime()
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    LCDwriteCommandByte(0x80)   #80=Top Line
    LCDwriteDataByte(ord(' '))
    if (hh < 10):
        LCDwriteDataByte(ord('0'))
    LCDshort(hh);
    LCDwriteDataByte(ord(':'))
    if (mm < 10):
        LCDwriteDataByte(ord('0'))
    LCDshort(mm)
    LCDwriteDataByte(ord(':'))
    if (ss < 10):
        LCDwriteDataByte(ord('0'))
    LCDshort(ss)
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('G'))
    LCDwriteDataByte(ord('M'))
    LCDwriteDataByte(ord('T'))
    LCDwriteCommandByte(0xC0)   #C0=Bottom Line
    LCDwriteDataByte(ord(' '))
    if (MM < 10):
        LCDwriteDataByte(ord('0'))
    LCDshort(MM)
    LCDwriteDataByte(ord('-'))
    if (dd < 10):
        LCDwriteDataByte(ord('0'))
    LCDshort(dd)
    LCDwriteDataByte(ord('-'))
    LCDshort(yy)
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('G'))
    LCDwriteDataByte(ord('M'))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord(' '))
    pyb.delay(3000)
#*********** LAT LONG ***********************
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord('L'),ord('A'),ord('T'),ord(':'),ord(' '),eeReadData[0],eeReadData[1],eeReadData[2],eeReadData[3],\
        eeReadData[4],eeReadData[5],eeReadData[6],eeReadData[7],eeReadData[8],ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 1)
    constructedLine = [ord('L'),ord('O'),ord('N'),ord(':'),ord(' '),eeReadData[9],eeReadData[10],eeReadData[11],eeReadData[12],\
        eeReadData[13],eeReadData[14],eeReadData[15],eeReadData[16],eeReadData[17],eeReadData[18],ord(' ')]
    LCDwriteLine(constructedLine, 0)
    pyb.delay(1000)

#*********** TAKE GPS? ***********************
    #constructedLine = [ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),\
    #    ord(''),ord(''),ord(''),ord(''),ord(''),ord(' '),ord(' ')]

    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord('G'),ord('E'),ord('T'),ord(' '),ord('T'),ord('I'),ord('M'),ord('E'),ord(' '),\
        ord('&'),ord(' '),ord('G'),ord('P'),ord('S'),ord('?'),ord(' ')]
    LCDwriteLine(constructedLine, 1)
    constructedLine = [ord('N'),ord('O'),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),\
        ord(' '),ord(' '),ord(' '),ord('Y'),ord('E'),ord('S'),ord(' ')]
    LCDwriteLine(constructedLine, 0)
    iters=0
    while (iters<300):
        iters+=1
        if (PBUTTON_L.value() == 1):   #USER P.BUTTON LEFT (EXIT)
            LCDwriteCommandByte(0x01)	#   #Clear
            pyb.delay(10)
            constructedLine = [ord('N'),ord('O'),ord(' '),ord('G'),ord('P'),ord('S'),ord(' '),ord(' '),ord(' '),\
                ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' ')]
            LCDwriteLine(constructedLine, 1)
            iters=300;
        else:
            if (PBUTTON_R.value() == 1):   #USER P.BUTTON RIGHT (GET GPS)
#                acqGPSwithLCD()
                getGPSwithLCD()
                iters=500
        pyb.delay(10)
    pyb.delay(1000)
#*********** TEST LIDAR? ***********************
    #constructedLine = [ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),\
    #    ord(''),ord(''),ord(''),ord(''),ord(''),ord(' '),ord(' ')]

    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord('T'),ord('E'),ord('S'),ord('T'),ord(' '),ord('L'),ord('I'),ord('D'),ord('A'),\
        ord('R'),ord('?'),ord(' '),ord(' '),ord(' '),ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 1)
    constructedLine = [ord('N'),ord('O'),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),\
        ord(' '),ord(' '),ord(' '),ord('Y'),ord('E'),ord('S'),ord(' ')]
    LCDwriteLine(constructedLine, 0)
    iters=0
    while (iters<300):
        iters+=1
        if (PBUTTON_L.value() == 1):   #USER P.BUTTON LEFT (EXIT)
            LCDwriteCommandByte(0x01)	#   #Clear
            pyb.delay(10)
#            constructedLine = [ord('N'),ord('O'),ord(' '),ord('G'),ord('P'),ord('S'),ord(' '),ord(' '),ord(' '),\
#                ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' '),ord(' ')]
#            LCDwriteLine(constructedLine, 1)
            iters=300;
        else:
            if (PBUTTON_R.value() == 1):   #USER P.BUTTON RIGHT (GET GPS)
                testLidar()
                iters=500
        pyb.delay(10)
    pyb.delay(1000)
#*********** SENSORS ***********************
    if(DO_WEATHER_SENSORS):
        SENS_PWRC.high() # or p.value(1) to make the pin high (3.3V)
        pyb.delay(50)
        humid = 22  #getHumidity(0x80)   #40
        als = 33    #getALSfromVEML(0x20)
        gotBaromTemperature=55  #getAltimeterTemperatureTE(0xEE);
        gotBarom = gotBaromTemperature & 0x0000FFFF
        gotTemperature = gotBaromTemperature>>16
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    LCDwriteCommandByte(0x80)   #80=Top Line
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord('='))
    LCDshort(gotTemperature)
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('H'))
    LCDwriteDataByte(ord('='))
    LCDshort(humid)
    LCDwriteCommandByte(0xC0)   #C0=Bottom Line
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('B'))
    LCDwriteDataByte(ord('='))
    LCDshort(gotBarom)
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('L'))
    LCDwriteDataByte(ord('='))
    LCDshort((als))
def write_sda_L (x):
    if(x):
        sensorSDA_INpull()
    else:
        sensorSDA_OUTlow()
def send_slave_Address_L(readBit, slaveAddress):
    return i2c_write_byte_L(slaveAddress | readBit, 0)
def twi_start_cond_L():
    sensorSCL_INpull()
    sensorSDA_OUTlow()
    pyb.delay(1)
    sensorSCL_OUTlow()
    pyb.delay(1)
def twi_stop_cond_L():
    sensorSDA_OUT()
    pyb.delay(1)	#delay_us(DELAY_L); was usec in Atmel
    sensorSCL_INpull()
    pyb.delay(1)	#delay_us(DELAY_L); was usec in Atmel
def sensorSCL_INpull():
    SENS_SCL.init(SENS_SCL.IN, SENS_SCL.PULL_UP)
def sensorSDA_INpull():
    SENS_SDA.init(SENS_SDA.IN, SENS_SDA.PULL_UP)
def sensorSDA_OUT():
    SENS_SDA.init(SENS_SDA.OUT_PP, SENS_SDA.PULL_NONE)
def sensorSCL_OUT():
    SENS_SCL.init(SENS_SCL.OUT_PP, SENS_SCL.PULL_NONE)
def sensorSDA_OUTlow():
    SENS_SDA.init(SENS_SDA.OUT_PP, SENS_SDA.PULL_NONE)
    SENS_SDA.value(0)
def sensorSCL_OUTlow():
    SENS_SCL.init(SENS_SCL.OUT_PP, SENS_SCL.PULL_NONE)
    SENS_SCL.value(0)
def samSDA_INpull():
    SAM_SDA.init(SAM_SDA.IN, SAM_SDA.PULL_UP)
def samSCL_INpull():
    SAM_SCL.init(SAM_SCL.IN, SAM_SCL.PULL_UP)
def samSDA_OUT():
    SAM_SDA.init(SAM_SDA.OUT_PP, SAM_SDA.PULL_NONE)
def samSCL_OUT():
    SAM_SCL.init(SAM_SCL.OUT_PP, SAM_SCL.PULL_NONE)
def samSCL_OUTlow():
    SAM_SCL.init(SAM_SCL.OUT_PP, SAM_SCL.PULL_NONE)
    SAM_SCL.value(0)
def samSDA_OUTlow():
    SAM_SDA.init(SAM_SDA.OUT_PP, SAM_SDA.PULL_NONE)
    SAM_SDA.value(0)

def read_bytes_L(data, numBytes, slaveAddress):
    index=0
    twi_start_cond_L()
    send_slave_Address_L(1, slaveAddress)
    for index in range (0, numBytes):
        i2c_read_byte_L(data, numBytes, index)
    twi_stop_cond_L()

def i2c_read_byte_L(data, numBytes, index):
    byte = 0
    bitnum = 0
    sensorSDA_INpull()
    for bitnum in range (0,8):
       sensorSCL_INpull()
       pyb.delay(1)     #delay_us(DELAY_L); was one usec with Atmel
       if SENS_SDA.value():
          byte |= (1 << (7- bitnum))
       sensorSCL_OUTlow()
       pyb.delay(1)     #delay_us(DELAY_L); was one usec with Atmel
    data[index] = byte
    if(index < (numBytes-1)):  #if not the last byte, ACK the slave
       sensorSDA_OUTlow()	#ACK
       sensorSCL_INpull()  #SCL high for 9th clock
       pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
       sensorSCL_OUTlow()  #SCL low after 9th clock
       pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
       sensorSDA_INpull()  #release SDA after ACK
    else: #send NACK on the last byte
       sensorSDA_INpull()  #NACK
       sensorSCL_INpull()  #SCL high for the 9th clock
       pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
       sensorSCL_OUTlow()  #SCL low after 9th clock
       pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
       sensorSCL_INpull()  #goes high for the 9th clock
       pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
       while(SENS_SCL.value()==0):
            pass

def write_data_L(indata, numBytes, slaveAddress, stretch):
    ack = 0
    index = 0
    twi_start_cond_L()
    if (send_slave_Address_L(0, slaveAddress))==0:
        return 0
    for index in range (0, numBytes):
        if (index==(numBytes-1)):
            ack = i2c_write_byte_L(indata[index], stretch)
        else:
            ack = i2c_write_byte_L(indata[index], 0)
        if ack == 0:
            break       #STOP
    if(stretch==0):
        sensorSCL_INpull()
    pyb.delay(1)        #delay_us(SCL_SDA_DELAY_L);  was one usec in Atmel
    sensorSDA_INpull()
    return ack

def i2c_write_byte_L(byte, stretch):
    for bitnum in range (0, 8):
        write_sda_L((byte & 0x80) != 0)
        pyb.delay(1)	#delay_us(DELAY_L); was one usec with Atmel
        sensorSCL_INpull()
        pyb.delay(1)	#delay_us(DELAY_L); was one usec with Atmel
        sensorSCL_OUTlow()
        byte <<= 1
        pyb.delay(1)	#delay_us(DELAY_L); was one usec with Atmel
    sensorSDA_INpull()
    sensorSCL_INpull()  #goes high for the 9th clock
    pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
    pyb.delay(1)        #delay_us(4); was FOUR usec with Atmel
    #Check for ack
    if ((stretch==0) and SENS_SDA.value()):    #(HAL_GPIO_ReadPin(GPIOB, 1<<11)):
#        print("NO ACK")
        twi_stop_cond_L()
#        if (PRINT_DEBUGS):
#            print("NO ACK")
        return 0        #If no ACK from slave, return 0.
    pyb.delay(1)        #delay_us(DELAY_L); was one usec with Atmel
    sensorSCL_OUTlow()  #end of byte with acknowledgment.
    sensorSDA_OUTlow()
    if(stretch):
        pyb.delay(1)
    if (stretch and SENS_SDA.value()):    #(HAL_GPIO_ReadPin(GPIOB, 1<<11)):
#        print("STRETCH NO ACK")
        twi_stop_cond_L()
#    elif stretch:
#        print("GOT STRETCH ACK")
    return 1

def testLidar():
    constructedLine = [0]*16
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord(' '),ord('T'),ord('E'),ord('R'),ord('T'),ord('I'),ord('N'),ord('G'),ord(' '),ord('L'),\
ord('D'),ord('A'),ord('R'),ord(' '),ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 1)
    constructedLine = [ord('N'),ord('O'),ord(' '),ord('D'),ord('E'),ord('T'),ord('E'),ord('C'),ord('T'),\
        ord('I'),ord('O'),ord('N'),ord('S'),ord(' '),ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 0)
    pyb.delay(3000)
    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord(' '),ord(' '),ord('P'),ord('L'),ord('E'),ord('A'),ord('S'),ord('E'),ord(' '),\
ord('W'),ord('A'),ord('I'),ord('T'),ord(' '),ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 1)
    LCDwriteCommandByte(0xC0)   #Bottom line
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('S'))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord('A'))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord('U'))
    LCDwriteDataByte(ord('S'))
    LCDwriteDataByte(ord(':'))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('.'))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    pyb.delay(3000)

def getLoc():
    gpsAcqTime=0
    latWholeStr = "none"
    latFracStr = "none"
    latSign = "none"
    longWholeStr = "none"
    longFracStr = "none"
    longSign = "none"
    gotA = 666
    while ((gotA == 666) and (gpsAcqTime < 1000)):
        if uart2.any():
            dataFromGPS = uart2.read()
            gpsAcqTime += 4
            if dataFromGPS:
                stringFromGPS = dataFromGPS.decode()
                uart5.write(stringFromGPS + "\r\n")
                if (("GNRMC" in stringFromGPS) and ("*" in stringFromGPS)):
                    rmcIndex = stringFromGPS.index("GNRMC")
                    asteriskIndex = stringFromGPS.index("*", rmcIndex+1)
                    subRMC = stringFromGPS[rmcIndex:asteriskIndex+3]
                    crcStr = stringFromGPS[asteriskIndex+1:asteriskIndex+3]
                    badAsciiHex = 0
                    for char in crcStr:
                        if( ord(char)<0x30 or ord(char)>0x46):
                            badAsciiHex += 1
                    if (badAsciiHex == 0):
                        crcStr = "0x" + crcStr
                        crcValue = int(crcStr, 16)
                        subRMC = subRMC[0:len(subRMC)-3]
                        uart5.write(subRMC + "\r\n")
                        gpsChecksum = 0
                        for char in subRMC:
                            gpsChecksum ^= ord(char)
                        if(crcValue == gpsChecksum):
                            uart5.write("GOOD CRC\r\n")
                            commaIndexA = subRMC.index(",")
                            commaIndexB = subRMC.index(",", commaIndexA+1)
                            statusStr = subRMC[commaIndexB+1:commaIndexB+2]
                            if(ord(statusStr[0]) == 0x41):
                                uart5.write("GOT STATUS A\r\n")
                                gotA = 0
                                commaIndexC = subRMC.index(",", commaIndexB+1)
                                commaIndexD = subRMC.index(".", commaIndexC+1)
                                commaIndexE = subRMC.index(",", commaIndexD+1)
                                commaIndexF = subRMC.index(",", commaIndexE+1)
                                commaIndexG = subRMC.index(".", commaIndexF+1)
                                commaIndexH = subRMC.index(",", commaIndexG+1)
                                commaIndexJ = subRMC.index(",", commaIndexH+1)
                                latWholeStr = subRMC[commaIndexC+1:commaIndexD]
                                latFracStr = subRMC[commaIndexD+1:commaIndexE]
                                latSign = subRMC[commaIndexE+1:commaIndexF]
                                longWholeStr = subRMC[commaIndexF+1:commaIndexG]
                                longFracStr = subRMC[commaIndexG+1:commaIndexH]
                                longSign = subRMC[commaIndexH+1:commaIndexJ]

                            else:
                                gotA = 666
                        else:
                            uart5.write("BAD CRC\r\n")
                    else:
                        uart5.write("BAD UART CAPTURE OF CRC STRING\r\n")
                else:
                    uart5.write("\r\nNO RMC\r\n")
        time.sleep(0.1)
    uart5.write("END RMC\r\n\r\n")
    return gpsAcqTime, latWholeStr, latFracStr, latSign, longWholeStr, longFracStr, longSign

def getDate():
    gpsAcqTime = 0
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))#use UBLOX_CHECKSUM.vi (LabView) to calc the two checksum chars.
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))

    uart2.write("$GNGNQ,ZDA*")
    uart2.write(ustruct.pack("<bbbb", 0x32, 0x32, 0x0D, 0x0A))
    pyb.delay(1500)
    gotDate = 666
    while ((gotDate == 666) and (gpsAcqTime < 1000)):
        uart2.write("$GNGNQ,ZDA*")
        uart2.write(ustruct.pack("<bbbb", 0x32, 0x32, 0x0D, 0x0A))
#        pyb.delay(2200)
        if uart2.any():
            dataFromGPS = uart2.read()
            gpsAcqTime += 4
            if dataFromGPS:
                stringFromGPS = dataFromGPS.decode()
                uart5.write(stringFromGPS + "\r\n")
                if (("GNZDA" in stringFromGPS) and ("*" in stringFromGPS)):
                    uart5.write(stringFromGPS + "\r\n")
                    rmcIndex = stringFromGPS.index("GNZDA")
                    if (("GNZDA" in stringFromGPS, rmcIndex) and ("*" in stringFromGPS, rmcIndex)):
                        rmcIndex = stringFromGPS.index("GNZDA", rmcIndex)
                        asteriskIndex = stringFromGPS.index("*", rmcIndex+1)
                        subZDA = stringFromGPS[rmcIndex:asteriskIndex+3]
                        crcStr = stringFromGPS[asteriskIndex+1:asteriskIndex+3]
                        badAsciiHex = 0
                        for char in crcStr:
                            if( ord(char)<0x30 or ord(char)>0x46):
                                badAsciiHex += 1
                        if (badAsciiHex == 0):
                            crcStr = "0x" + crcStr
                            crcValue = int(crcStr, 16)
                            subZDA = subZDA[0:len(subZDA)-3]
                            uart5.write(subZDA + "\r\n")
                            gpsChecksum = 0
                            for char in subZDA:
                                gpsChecksum ^= ord(char)
                            if(crcValue == gpsChecksum):
                                uart5.write("GOOD CRC\r\n")
                                commaIndexA = subZDA.index(",")
                                commaIndexB = subZDA.index(",", commaIndexA+1)
                                statusStr = subZDA[commaIndexB+1:commaIndexB+2]
                                uart5.write(str(commaIndexA) + " " + str(commaIndexB) + " STATUS STRING: " + statusStr + " " + str(ord(statusStr[0])) + "\r\n")
                                if((ord(statusStr[0]) != ',') and (ord(statusStr[0]) != 'X')) :
                                    uart5.write("GOT DATE\r\n")
                                    gotDate = 0
                                else:
                                    gotDate = 666
                            else:
                                uart5.write("BAD CRC\r\n")
                        else:
                            uart5.write("BAD UART CAPTURE OF CRC STRING\r\n")
                else:
                    uart5.write("\r\nNO ZDA\r\n")
        time.sleep(1.2)
    if (gotDate == 0):
        dayStr = subZDA[commaIndexB+1:commaIndexB+3]
        monthStr = subZDA[commaIndexB+4:commaIndexB+6]
        yearStr = subZDA[commaIndexB+7:commaIndexB+11]
    uart5.write("END ZDA\r\n\r\n")
    return gpsAcqTime, dayStr, monthStr, yearStr

def getTime():
    gpsAcqTime = 0
    uart2.write("$PUBX,40,ZDA,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x34, 0x0D, 0x0A))#use UBLOX_CHECKSUM.vi (LabView) to calc the two checksum chars.
    uart2.write("$PUBX,40,ZDA,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x34, 0x0D, 0x0A))
    uart2.write("$PUBX,40,ZDA,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x34, 0x0D, 0x0A))
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))#use UBLOX_CHECKSUM.vi (LabView) to calc the two checksum chars.
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))
    uart2.write("$PUBX,40,RMC,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x37, 0x0D, 0x0A))

    pyb.delay(2000)
    gotTime = 666
    while ((gotTime == 666) and (gpsAcqTime < 1000)):
        uart2.write("$GNGNQ,GNS*")
        pyb.delay(1200)
        uart2.write(ustruct.pack("<bbbb", 0x32, 0x37, 0x0D, 0x0A))
        if uart2.any():
            dataFromGPS = uart2.read()
            gpsAcqTime += 4
            if dataFromGPS:
                stringFromGPS = dataFromGPS.decode()
                if (("GNGNS" in stringFromGPS) and ("*" in stringFromGPS)):
                    uart5.write(stringFromGPS + "\r\n")
                    rmcIndex = stringFromGPS.index("GNGNS")
                    if (("GNGNS" in stringFromGPS, rmcIndex) and ("*" in stringFromGPS, rmcIndex)):
                        rmcIndex = stringFromGPS.index("GNGNS", rmcIndex+1)
                        asteriskIndex = stringFromGPS.index("*", rmcIndex+1)
                        subGNS = stringFromGPS[rmcIndex:asteriskIndex+3]
                        crcStr = stringFromGPS[asteriskIndex+1:asteriskIndex+3]
                        badAsciiHex = 0
                        for char in crcStr:
                            if( ord(char)<0x30 or ord(char)>0x46):
                                badAsciiHex += 1
                        if (badAsciiHex == 0):
                            crcStr = "0x" + crcStr
                            crcValue = int(crcStr, 16)
                            subGNS = subGNS[0:len(subGNS)-3]
                            uart5.write(subGNS + "\r\n")
                            gpsChecksum = 0
                            for char in subGNS:
                                gpsChecksum ^= ord(char)
                            if(crcValue == gpsChecksum):
                                uart5.write("GOOD CRC\r\n")
                                commaIndexA = subGNS.index(",")
                                statusStr = subGNS[commaIndexA+1:commaIndexA+2]
                                if((ord(statusStr[0]) != ',') and (ord(statusStr[0]) != 'X')) :
                                    uart5.write("GOT TIME\r\n")
                                    gotTime = 0
                                else:
                                    gotTime = 666
                            else:
                                uart5.write("BAD CRC\r\n")
                        else:
                            uart5.write("BAD UART CAPTURE OF CRC STRING\r\n")
                else:
                    uart5.write("\r\nNO GNS\r\n")
        time.sleep(1.2)
        uart5.write("GPS ACQ TIME: " + str(gpsAcqTime) + "\r\n")
    if (gotTime == 0):
        hourStr = subGNS[commaIndexA+1:commaIndexA+3]
        minStr = subGNS[commaIndexA+3:commaIndexA+5]
        secStr = subGNS[commaIndexA+5:commaIndexA+7]
        uart5.write(hourStr + " " + minStr + " " + secStr + "\r\n")
    uart5.write("END GNS\r\n\r\n")
    return gpsAcqTime, hourStr, minStr, secStr

def getGPSwithLCD():
    totalAcqTime = 0
    gpsParams =[0]*10

    openGPS()
    initGPS()
    initGPS()
    initGPS()

    for iters in range (0, 2, 1):
        acqTime, latWholeStr, latFracStr, latSign, longWholeStr, longFracStr, longSign = getLoc()
        totalAcqTime += acqTime
        uart5.write(latWholeStr + "." + latFracStr + latSign + " " + longWholeStr + "." + longFracStr + longSign + "\r\n\r\n")
        gpsParams[0] = int(latWholeStr[0:2])
        if (latSign == 'N'):
            gpsParams[0] += 128
        gpsParams[1] = int(latWholeStr[2:4])
        gpsParams[2] = int(latFracStr[0:2])
        gpsParams[3] = int(latFracStr[2:3])
        if (longSign == 'E'):
            gpsParams[4] = '+'
        else:
            gpsParams[4] = '-'
        gpsParams[5] = int(longWholeStr[0:3])
        gpsParams[6] = int(longWholeStr[3:5])
        gpsParams[7] = int(longFracStr[0:2])
        gpsParams[8] = int(longFracStr[2:3])

        uart5.write(str(gpsParams[0]) + "." + str(gpsParams[1]) + str(gpsParams[2]) + str(gpsParams[3]) + "\r\n")
        uart5.write(str(gpsParams[5]) + "." + str(gpsParams[6]) + str(gpsParams[7]) + str(gpsParams[8]) + "\r\n")

#    rtc.datetime((packetFromSam[9]+2000, packetFromSam[10], packetFromSam[11], 1, packetFromSam[12], packetFromSam[13], packetFromSam[14], 0))
    asciiLatLong=bytearray()
    asciiLatLong = [0]*19

    compressedHi=0
    compressedLo=0
    if((gpsParams[0] & 0x80) == 0x80):
        asciiLatLong[0] = 0x2B  #'+'
    else:
        asciiLatLong[0] = 0x2D   #'-'
    asciiTemp = displayDecimal(gpsParams[0]&0x7F)
    asciiLatLong[1]=asciiTemp[3]    #Lat whole, high decimal digit
    asciiLatLong[2]=asciiTemp[4]
    asciiLatLong[3] = 0x2E  #'.'
    compressedHi |= (gpsParams[0]&0x7F) << 23

    latFrac = (int(((gpsParams[1]) * 10000)/6)) + (int((gpsParams[2]*100/6))) + int(gpsParams[3]/6)
    compressedHi |= (latFrac << 6) & 0x007FFFC0
    asciiTemp = displayDecimal(int(latFracStr))
    asciiLatLong[4]=asciiTemp[0]
    asciiLatLong[5]=asciiTemp[1]
    asciiLatLong[6]=asciiTemp[2]
    asciiLatLong[7]=asciiTemp[3]
    asciiLatLong[8]=asciiTemp[4]

    if (gpsParams[4] == '-'):
        asciiLatLong[9] = 0x2D
        compressedHi |= 0x00000020
    else:
        asciiLatLong[9] = 0x2B

#    asciiLatLong[9] = (gpsParams[4]  #Long Sign: SAM sends '+' or '-' in packetFromSam[4]

#if(gpsParams[4] == 0x2D):   #'-'

    asciiTemp = displayDecimal(gpsParams[5])
    asciiLatLong[10]=asciiTemp[2]
    asciiLatLong[11]=asciiTemp[3]
    asciiLatLong[12]=asciiTemp[4]
    asciiLatLong[13] = 0x2E #'.'
    compressedHi |= 0x70 >> 3  #gpsParams[5] >> 3
    compressedLo |= 0x01  #  gpsParams[5] << 29

    longFrac = (int((gpsParams[6] *10000)/6)) + (int((gpsParams[7]*100/6))) + (int(gpsParams[8]/6))
    compressedLo |= (longFrac << 12) & 0x1FFFF000
    asciiTemp = displayDecimal(int(longFracStr))
    asciiLatLong[14]=asciiTemp[0]
    asciiLatLong[15]=asciiTemp[1]
    asciiLatLong[16]=asciiTemp[2]
    asciiLatLong[17]=asciiTemp[3]
    asciiLatLong[18]=asciiTemp[4]

    uart5.write(chr(asciiLatLong[0]) + chr(asciiLatLong[1]) + chr(asciiLatLong[2])  + chr(asciiLatLong[3])  + chr(asciiLatLong[4])  + chr(asciiLatLong[5])\
    + chr(asciiLatLong[6]) + chr(asciiLatLong[7]) + chr(asciiLatLong[8]) + chr(asciiLatLong[9]) + chr(asciiLatLong[10]) + chr(asciiLatLong[11])\
    + chr(asciiLatLong[12])  + chr(asciiLatLong[13])  + chr(asciiLatLong[14])  + chr(asciiLatLong[15])  + chr(asciiLatLong[16])  + chr(asciiLatLong[17])  + chr(asciiLatLong[18])+"\r\n"   )

    eepromSDA_OUT()
    EEPROM_SDA.value(1)
    pyb.delay(50)

    #bID = bytearray()  #declares bID as a bytearray
    #bID.extend(uniqueID.encode())

    eepromWriteAsciiLatLong(asciiLatLong)
    eepromWriteCompressedLatLong(compressedHi, 0xDC)
    eepromWriteCompressedLatLong(compressedLo, 0xE0)

    dateTime = rtc.datetime()
    uart5.write(str(dateTime[4])+":"+str(dateTime[5])+":"+str(dateTime[6])+"\r\n")

    LCDwriteCommandByte(0x01)   #Clear
    pyb.delay(10)
    constructedLine = [ord(' '),ord(' '),ord('G'),ord('P'),ord('S'),ord(' '),ord('S'),ord('U'),ord('C'),ord('C'),\
        ord('E'),ord('S'),ord('S'),ord('!'),ord(' '),ord(' ')]
    LCDwriteLine(constructedLine, 1)

    LCDwriteCommandByte(0xC0)   #C0=bottom line
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))
    if(dateTime[4]<10):
        LCDwriteDataByte(ord('0'))
    LCDshort(dateTime[4]);
    LCDwriteDataByte(ord(':'))
    if(dateTime[5]<10):
        LCDwriteDataByte(ord('0'))
    LCDshort(dateTime[5]);
    LCDwriteDataByte(ord(':'))
    if(dateTime[6]<10):
        LCDwriteDataByte(ord('0'))
    LCDshort(dateTime[6]);
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord('G'))
    LCDwriteDataByte(ord('M'))
    LCDwriteDataByte(ord('T'))
    LCDwriteDataByte(ord(' '))
    LCDwriteDataByte(ord(' '))

    for iters in range (0, 2, 1):
        acqTime, dayStr, monthStr, yearStr = getDate()
        totalAcqTime += acqTime
        uart5.write(monthStr + "-" + dayStr + "-" + yearStr + "   TotalAcqTime: " + str(totalAcqTime) + "\r\n\r\n")

    for iters in range (0, 2, 1):
        acqTime, hourStr, minStr, secStr = getTime()
        totalAcqTime += acqTime
        uart5.write(hourStr + ":" + minStr + ":" + secStr + "   TotalAcqTime: " + str(totalAcqTime) + "\r\n\r\n")

    uart5.write("TOTAL ACQ TIME: " + str(totalAcqTime) + "\r\n")
    uart5.write("END GET GPS()\r\n")

    closeGPS()

#def clearGPSparams():

#def tryUblox():

def closeGPS():
    GPS_PWRC = pyb.Pin("K5", pyb.Pin.IN)
    GPS_PWRC.PULL_NONE

def openGPS():
    GPS_PWRC = pyb.Pin("K5", pyb.Pin.OUT_PP)
    GPS_PWRC.high()

def initGPS():
    pyb.delay(1000)
    uart2.write("$PUBX,40,GLL,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x35, 0x43, 0x0D, 0x0A))#use UBLOX_CHECKSUM.vi (LabView) to calc the two checksum chars.
    uart2.write("$PUBX,40,GSA,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x34, 0x45, 0x0D, 0x0A))
    uart2.write("$PUBX,40,GSV,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x35, 0x39, 0x0D, 0x0A))
    uart2.write("$PUBX,40,GGA,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x35, 0x41, 0x0D, 0x0A))
    uart2.write("$PUBX,40,VTG,1,0,1,1,1,0*")
    uart2.write(ustruct.pack("<bbbb", 0x35, 0x45, 0x0D, 0x0A))


def acqGPSwithLCD():
    #constructedLine = [ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),ord(''),\
    #    ord(''),ord(''),ord(''),ord(''),ord(''),ord(' '),ord(' ')]
    packetFromSam = [0]*64
    askSAMforGPS(packetFromSam, 1)  #1=initiate a new GPS acqquisition.
    while(packetFromSam[28]<5):
        LCDwriteCommandByte(0x01)   #Clear
        pyb.delay(10)
        constructedLine = [ord(' '),ord('A'),ord('C'),ord('Q'),ord('I'),ord('R'),ord('I'),ord('N'),ord('G'),ord(' '),\
            ord('G'),ord('P'),ord('S'),ord(' '),ord(' '),ord(' ')]
        LCDwriteLine(constructedLine, 1)
        constructedLine = [ord(' '),ord('E'),ord('X'),ord('P'),ord('O'),ord('S'),ord('E'),ord(' '),ord('T'),\
            ord('O'),ord(' '),ord('S'),ord('K'),ord('Y'),ord(' '),ord(' ')]
        LCDwriteLine(constructedLine, 0)
        pyb.delay(3000)
        LCDwriteCommandByte(0x01)   #Clear
        pyb.delay(10)
        constructedLine = [ord(' '),ord(' '),ord('P'),ord('L'),ord('E'),ord('A'),ord('S'),ord('E'),ord(' '),\
           ord('W'),ord('A'),ord('I'),ord('T'),ord(' '),ord(' '),ord(' ')]
        LCDwriteLine(constructedLine, 1)
        LCDwriteCommandByte(0xC0)   #Bottom line
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord('S'))
        LCDwriteDataByte(ord('T'))
        LCDwriteDataByte(ord('A'))
        LCDwriteDataByte(ord('T'))
        LCDwriteDataByte(ord('U'))
        LCDwriteDataByte(ord('S'))
        LCDwriteDataByte(ord(':'))
        LCDwriteDataByte(ord(' '))
        LCDshort(packetFromSam[28]);
        LCDwriteDataByte(ord('.'))
        LCDshort(packetFromSam[29]);
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        pyb.delay(3000)
        askSAMforGPS(packetFromSam, 0)  #0=ask for status

    if (packetFromSam[28] == 5):
        print(" packetFrmSam[29] = %X" % (packetFromSam[29]))
#        if (packetFromSam[29] > 5): #tshooooooooooooooooooooooooooooot
        rtc.datetime((packetFromSam[9]+2000, packetFromSam[10], packetFromSam[11], 1, packetFromSam[12], packetFromSam[13], packetFromSam[14], 0))
        asciiLatLong=bytearray()
        asciiLatLong = [0]*19

        compressedHi=0
        compressedLo=0
        if((packetFromSam[0] & 0x80) == 0x80):
            asciiLatLong[0] = 0x2B  #'+'
        else:
            asciiLatLong[0] = 0x2D   #'-'
        asciiTemp = displayDecimal(packetFromSam[0]&0x7F)
        uart5.write(str(asciiTemp[0])+" ")
        uart5.write(str(asciiTemp[1])+" ")
        uart5.write(str(asciiTemp[2])+" ")
        uart5.write(str(asciiTemp[3])+" ")
        uart5.write(str(asciiTemp[4])+"\r\n")
        asciiLatLong[1]=asciiTemp[3]    #Lat whole, high decimal digit
        asciiLatLong[2]=asciiTemp[4]
        asciiLatLong[3] = 0x2E  #'.'
        compressedHi |= (packetFromSam[0]&0x7F) << 23

        latFrac = (int(((packetFromSam[1]) * 10000)/6)) + (int((packetFromSam[2]*100/6))) + int(packetFromSam[3]/6)
        compressedHi |= (latFrac << 6) & 0x007FFFC0
        asciiTemp = displayDecimal(latFrac)
        asciiLatLong[4]=asciiTemp[0]
        asciiLatLong[5]=asciiTemp[1]
        asciiLatLong[6]=asciiTemp[2]
        asciiLatLong[7]=asciiTemp[3]
        asciiLatLong[8]=asciiTemp[4]
        asciiLatLong[9] = packetFromSam[4]  #Long Sign: SAM sends '+' or '-' in packetFromSam[4]
        if(packetFromSam[4] == 0x2D):   #'-'
            compressedHi |= 0x00000020

        asciiTemp = displayDecimal(packetFromSam[5])
        asciiLatLong[10]=asciiTemp[2]
        asciiLatLong[11]=asciiTemp[3]
        asciiLatLong[12]=asciiTemp[4]
        asciiLatLong[13] = 0x2E #'.'
        compressedHi |= packetFromSam[5] >> 3
        compressedLo |= packetFromSam[5] << 29

        longFrac = (int((packetFromSam[6] *10000)/6)) + (int((packetFromSam[7]*100/6))) + (int(packetFromSam[8]/6))
        compressedLo |= (longFrac << 12) & 0x1FFFF000
        asciiTemp = displayDecimal(longFrac)
        asciiLatLong[14]=asciiTemp[0]
        asciiLatLong[15]=asciiTemp[1]
        asciiLatLong[16]=asciiTemp[2]
        asciiLatLong[17]=asciiTemp[3]
        asciiLatLong[18]=asciiTemp[4]

        uart5.write(chr(asciiLatLong[0]) + chr(asciiLatLong[1]) + chr(asciiLatLong[2])  + chr(asciiLatLong[3])  + chr(asciiLatLong[4])  + chr(asciiLatLong[5])\
+ chr(asciiLatLong[6]) + chr(asciiLatLong[7])  + chr(asciiLatLong[8])  + chr(asciiLatLong[9])  + chr(asciiLatLong[10]) + chr(asciiLatLong[11])  + chr(asciiLatLong[12])  + chr(asciiLatLong[13])  + chr(asciiLatLong[14])  + chr(asciiLatLong[15])  + chr(asciiLatLong[16])  + chr(asciiLatLong[17])  + chr(asciiLatLong[18])+"\r\n"   )

        eepromSDA_OUT()
        EEPROM_SDA.value(1)
        pyb.delay(50)

 #bID = bytearray()  #declares bID as a bytearray
 #bID.extend(uniqueID.encode())

        eepromWriteAsciiLatLong(asciiLatLong)
        eepromWriteCompressedLatLong(compressedHi, 0xDC)
        eepromWriteCompressedLatLong(compressedLo, 0xE0)

        dateTime = rtc.datetime()
        uart5.write(str(dateTime[4])+":"+str(dateTime[5])+":"+str(dateTime[6])+"\r\n")

        LCDwriteCommandByte(0x01)   #Clear
        pyb.delay(10)
        constructedLine = [ord(' '),ord(' '),ord('G'),ord('P'),ord('S'),ord(' '),ord('S'),ord('U'),ord('C'),ord('C'),\
            ord('E'),ord('S'),ord('S'),ord('!'),ord(' '),ord(' ')]
        LCDwriteLine(constructedLine, 1)

        LCDwriteCommandByte(0xC0)   #C0=bottom line
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
        if(dateTime[4]<10):
            LCDwriteDataByte(ord('0'))
        LCDshort(dateTime[4]);
        LCDwriteDataByte(ord(':'))
        if(dateTime[5]<10):
            LCDwriteDataByte(ord('0'))
        LCDshort(dateTime[5]);
        LCDwriteDataByte(ord(':'))
        if(dateTime[6]<10):
            LCDwriteDataByte(ord('0'))
        LCDshort(dateTime[6]);
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord('G'))
        LCDwriteDataByte(ord('M'))
        LCDwriteDataByte(ord('T'))
        LCDwriteDataByte(ord(' '))
        LCDwriteDataByte(ord(' '))
    pyb.delay(1000)

#        displayMenu()



def askSAMforGPS(packFromSam, reqGPS):
    ASK_FOR_GPS_STATUS = 2
    INITIATE_GSP_ACQ = 3
    samRes=14
    samResTotal=0
    for iters in range (3, 0, -1):
        samSDA_INpull()
        pyb.delay(1)
        if(reqGPS>0):
            samRes = sendPacketToSAM(INITIATE_GSP_ACQ)
        else:
            samRes = sendPacketToSAM(ASK_FOR_GPS_STATUS)
        pyb.delay(2)   #give Tiny time to calc CRC
        getPacketFromSAM(packFromSam)
        samSDA_INpull()
        crcrc=calcCRC(packFromSam,62)
        crcLoByte=crcrc&0x00FF
        crcHiByte=crcrc>>8
        samRes=0
        if(crcHiByte!=packFromSam[62]):
            samRes += 1
        if(crcLoByte!=packFromSam[63]):
            samRes += 1
        if(samRes==0):
            pyb.delay(100)  #May 13, 2024: Works at 150,300,500.   Fails at 50, 100.
            break
        pyb.delay(500)
    samResTotal += samRes
    samResTotal+=samRes;
    return samRes

def askSAMforMotion(packFromSam):
    ASK_FOR_GPS_STATUS = 2
    INITIATE_GSP_ACQ = 3
    samRes=14
    samResTotal=0
    for iters in range (30, 0, -1):
        samSDA_INpull()
        pyb.delay(1)    #intm crc failures without this May 2023 in K-World C.
        dateTime = rtc.datetime()
        if (dateTime[5] == 9):
            sendPacketToSAM(4)
        else:
            sendPacketToSAM(1)
        pyb.delay(2)    #time for SAM to calc crc. March 2024: 1ms is too fast. 2 works.
        getPacketFromSAM(packFromSAM)
        samSDA_INpull()
        crcrc=calcCRC(packFromSam,62)
        crcLoByte=crcrc&0x00FF
        crcHiByte=crcrc>>8
        samRes=0
        if(crcHiByte!=packFromSam[62]):
            samRes += 1
        if(crcLoByte!=packFromSam[63]):
            samRes += 1
        if(samRes==0):
            pyb.delay(150)  #May 13, 2024: Works at 150,300,500.   Fails at 50, 100.
            sendPacketToSAM(5) #packet(5) is ACK to SAM that recieved a good motion packet, so okay for SAM to clear previous motParams.
            break
        pyb.delay(500)
        samResTotal += samRes
    samResTotal+=samRes;
    return samRes

def getPacketFromSAM(packFromSam):
    Sam_CLK_DELAY = 1
    byteInProgress = 0;
    samSDA_INpull();
    samSCL_OUTlow();
    for numBytes in range (0, 64):
        byteInProgress = 0;
        for bit in range (0, 8):
            samSCL_OUT()
            SAM_SCL.value(1)
            pyb.udelay(Sam_CLK_DELAY)
            if (SAM_SDA.value()):
                byteInProgress |= (1<<(7-bit))
            pyb.udelay(Sam_CLK_DELAY)
            samSCL_OUTlow()
            pyb.udelay(Sam_CLK_DELAY)
        packFromSam[numBytes]=byteInProgress;
    samSCL_INpull()
    samSDA_INpull()    #End ACK
    pyb.udelay(Sam_CLK_DELAY)
    return 0

def sendPacketToSAM(argForSam):
    minPacketForSam = [0]*16
    Sam_TIMEOUT = 200000	#misses first of five at 10,000
    comStatus = 0
    samSCL_INpull()
    samSDA_INpull()
    WU_SAM.value(1)     #PA3=UnWakeUp Sam
    WU_SAM.init(WU_SAM.OUT_PP, WU_SAM.PULL_NONE)
    timeoutCtr=Sam_TIMEOUT
    WU_SAM.value(0)     #PA3=0=WakeUp Sam
    while(((SAM_SDA.value())==1)and(timeoutCtr>0)):
        timeoutCtr -= 1  #Wait for SDA to go low.
    timeoutCtr=Sam_TIMEOUT;
    while(((SAM_SDA.value())==0)and(timeoutCtr>0)):
        timeoutCtr -= 1  #Wait for SDA to go high.
    WU_SAM.value(1)     #PA3=UnWakeUp Sam
    minPacketForSam[0]=ord('F')         #firstByte 'F';//
    minPacketForSam[1]=argForSam  #01=, 02=ask for GPS status
    minPacketForSam[2]=SAM_USE_AUTO_LIDAR   #0x01 = use auto. #0x02 = reset daily counter for max num of pics
    minPacketForSam[3]=SAM_USE_FIXED_MIN
    minPacketForSam[4]=SAM_USE_FIXED_MAX >> 8
    minPacketForSam[5]=SAM_USE_FIXED_MAX & 0x00FF
    minPacketForSam[6]=SAM_MAX_DAILY_MOTION_PICS >> 8
    minPacketForSam[7]=SAM_MAX_DAILY_MOTION_PICS & 0x00FF
    minPacketForSam[8]=0
    minPacketForSam[9]=0
    minPacketForSam[10]=0
    minPacketForSam[11]=0
    minPacketForSam[12]=0
    minPacketForSam[13]=0
    crcrcrc=calcCRC(bytearray(minPacketForSam),14)
    minPacketForSam[14]=crcrcrc>>8
    minPacketForSam[15]=crcrcrc&0x00FF
    comStatus = 0
    for i in range (0, 16):
        comStatus |= sendCmdToSam(minPacketForSam[i])
    samSCL_OUT()
    SAM_SCL.value(1)
#    print("%X %X %X %X" % (minPacketForSam[0], minPacketForSam[1], minPacketForSam[14], minPacketForSam[15]))
#    print("comStatus= %d" % (comStatus))
    return comStatus

def sendCmdToSam(cmdToSend):
    SAM_SCL.value(1)
    SAM_SCL.value(1)
    Sam_CLK_DELAY = 1
    samSDA_OUT()
    samSCL_OUTlow()
    for bit in range (0, 8):
        SAM_SDA.value((cmdToSend & 0x80) != 0)
        pyb.udelay(Sam_CLK_DELAY)
        samSCL_OUT()
        SAM_SCL.value(1)
        pyb.udelay(Sam_CLK_DELAY)
        SAM_SCL.value(0)
        if (bit==7):
            samSDA_INpull()    #release SDA to allow for ACK from slave
        cmdToSend <<= 1
        pyb.udelay(Sam_CLK_DELAY)
    pyb.udelay(Sam_CLK_DELAY)
    if(SAM_SDA.value()):
        samSCL_INpull()    #and wait for SCL to go HIGH
        return 1            #NO ACK
    pyb.udelay(Sam_CLK_DELAY)
    return 0

def displayDecimal(a):
    asciiArray = [0x20] * 5
    if(a>99999):
        a=99999
    tenThou=int(a/10000)
    a=a%10000
    thou=int(a/1000)
    a=a%1000
    hund=int(a/100)
    a=a%100
    tens=int(a/10)
    a=a%10
    if(tenThou>0):
        asciiArray[0]=0x30 + tenThou
    if((thou>0) or (tenThou>0)):
        asciiArray[1]=0x30 + thou
    if((hund>0) or (thou>0) or (tenThou>0)):
        asciiArray[2]=0x30 + hund
    if((tens>0) or (hund>0) or (thou>0) or (tenThou>0)):
        asciiArray[3]=0x30 + tens
    asciiArray[4]=0x30 + a
    return asciiArray

def getALSfromVEML(slaveAddress):
    alsData = [0,0,0,0]
    iters = 0
    lightMeasurement = 0
    sensorSCL_OUTlow()
    sensorSDA_OUTlow()
    alsData[0]=0x01    #Register Address 01
    alsData[1]=0x00    #Config:  High threshold window
    alsData[2]=0x00
    write_data_L(alsData,3,slaveAddress,0)
    alsData[0]=0x02    #Register Address 02
    alsData[1]=0x00    #Config:  Low threshold window
    alsData[2]=0x00
    write_data_L(alsData,3,slaveAddress,0)
    alsData[0]=0x03    #Register Address 03
    alsData[1]=0x00    #Power saving mode
    alsData[2]=0x00
    write_data_L(alsData,3,slaveAddress,0)
    alsData[0]=0x00    #Register Address 00
    alsData[1]=0x00    #Config:  0x000 00(sensitivity=1) 0 00(integration time=100ms)
    alsData[2]=0x00    #00(integration time) 00(persistence) 00 0(no interrupt) 0(power on)
    write_data_L(alsData,3,slaveAddress,0)
    pyb.delay(100)
    alsData[0]=0x04    #
    write_data_L(alsData,1,slaveAddress,1)
    sensorSCL_INpull()
    sensorSDA_INpull()
    for inters in range (0,3):
        alsData[iters]=0    #ZERO the buffer
    read_bytes_L(alsData,2,slaveAddress)
    sensorSCL_INpull()
    sensorSDA_INpull()
    lightMeasurement = (alsData[1] << 4) + (alsData[0]>>4)
#    if(PRINT_DEBUGS):
#        print("End ALS %d" % lightMeasurement)
    return(lightMeasurement)

def getHumidity (slaveAddr):#HOLDS MASTER'S SCL LINE DOWN FOR ABOUT 10ms.
    sensorSCL_INpull()
    sensorSDA_INpull()
    pyb.delay(10)
    sensorSDA_OUTlow()      #Make them both low so that when the code drives them they will be low.
    dataOut = [0xE5,0,0]    #Config. Trigger humidity measurement and Hold Master.
    write_data_L(dataOut,1,slaveAddr,0)
    pyb.delay(10)
    dataIn=[0,0,0]
    read_bytes_L(dataIn,3,slaveAddr)
    humidityMeasurement=((dataIn[0])<<2)+((dataIn[1])>>6)   #only want the upper ten bits
    sensorSCL_INpull()
    sensorSDA_INpull()
    if(PRINT_DEBUGS):
        print("End Humidity     %d %d    %d" % (dataIn[0], dataIn[1]>>6, humidityMeasurement))
    return humidityMeasurement

def getTemperature (slaveAddr): #HOLDS MASTER'S SCL LINE DOWN FOR ABOUT 40ms
    sensorSCL_INpull()
    sensorSDA_INpull()
    pyb.delay(10)
    sensorSDA_OUTlow()      #Make them both low so that when the code drives them they will be low.
    dataOut = [0xE3,0,0]    #Config. Trigger temperature measurement and don't hold Master.
    write_data_L(dataOut,1,slaveAddr,0)
    pyb.delay(50)
    dataIn=[0,0,0]
    read_bytes_L(dataIn,3,slaveAddr)
    temperatureMeasurement=(((dataIn[0])<<4)+((dataIn[1])>>4))   #only want the upper ten bits
    if(PRINT_DEBUGS):
        print("End Temperature %d %d   %d" % (dataIn[0], dataIn[1]>>4, temperatureMeasurement))
    sensorSCL_INpull()
    sensorSDA_INpull()
    return temperatureMeasurement


def sendLRW():
    SENS_PWRC.high() # will be needed when calls buildLRWpacket()
    LRW_PWRC.low()
    pyb.delay(200)  #seems to work at 100 low then 300 high. Might be able to shave more off. TBD.  50 / 100 worked
    LRW_PWRC.high()
    pyb.delay(100)
    buildLRWpacket()   #(fakeBytes,testLoopCtr);
    uart2.write("FD")
    uart2.write(ustruct.pack("<b", LRW_TIMING_OFFSET))   #xmt offset
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[0], packetToLRW[1], packetToLRW[2], packetToLRW[3], packetToLRW[4]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[5], packetToLRW[6], packetToLRW[7], packetToLRW[8], packetToLRW[9]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[10], packetToLRW[11], packetToLRW[12], packetToLRW[13], packetToLRW[14]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[15], packetToLRW[16], packetToLRW[17], packetToLRW[18], packetToLRW[19]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[20], packetToLRW[21], packetToLRW[22], packetToLRW[23], packetToLRW[24]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[25], packetToLRW[26], packetToLRW[27], packetToLRW[28], packetToLRW[29]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[30], packetToLRW[31], packetToLRW[32], packetToLRW[33], packetToLRW[34]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[35], packetToLRW[36], packetToLRW[37], packetToLRW[38], packetToLRW[39]))
    uart2.write(ustruct.pack("<bbbbb", packetToLRW[40], packetToLRW[41], packetToLRW[42], packetToLRW[43], packetToLRW[44]))
    uart2.write(ustruct.pack("<bbbbbb", packetToLRW[45], packetToLRW[46], packetToLRW[47], packetToLRW[48], packetToLRW[49], packetToLRW[50]))
    uart2.write("T")

def buildLRWpacket():
    gotTemperature = getTemperature(0x80)
    als=getALSfromVEML(0x20)
    SENS_PWRC.low() # or p.value(1) to make the pin high (3.3V)
    eeReadData = [0]*4
    yy,MM,dd,ww,hh,mm,ss,uu = rtc.datetime()
    packetToLRW[0] = (LRW_FORMAT << 4) + ((yy%10) & 0x0F)
    packetToLRW[1] = (MM << 4) + (dd >> 1)
    packetToLRW[2] = (dd << 7) + (hh << 2) + ((mm & 0x30) >> 4)
    read_bytes_EE(eeReadData,4,0xF7,0xA2)
    packetToLRW[3] = ((mm & 0x0F) << 4) + (eeReadData[0] & 0x0000000F)
    packetToLRW[4] = 3  #eeReadData[1]  TSHOOOOOOOOOOOOOOOOOOOOOT temporarily setting deployment number to 3 as a test.
    read_bytes_EE(eeReadData,4,0xDC,0xA2)
    packetToLRW[5] =  eeReadData[0]
    packetToLRW[6] =  eeReadData[1]
    packetToLRW[7] =  eeReadData[2]
    packetToLRW[8] =  eeReadData[3]
    read_bytes_EE(eeReadData,4,0xE0,0xA2)
    packetToLRW[9] =  eeReadData[0]
    packetToLRW[10] = eeReadData[1]
    packetToLRW[11] = eeReadData[2] & 0xF0
    packetToLRW[11] += (gotTemperature >> 8) & 0x000F
    packetToLRW[12] = gotTemperature & 0x00FF
    packetToLRW[13] = als & 0x00FF
    packetToLRW[14] = numMotTriggers


    fileOffset = (((hh % 2)*60) + mm) << 5
    if(fileOffset < 2): #shift the pic segments by two minutes so that taking a new pic at XX:01 doesn't interfere with sending a new header.
        fileOffset = 118 + fileOffset
    else:
        fileOffset = fileOffset - 2
    packetToLRW[15] = fileOffset
    packetToLRW[16] = 3840>>5

    lrwFilename = "LRW_PIC.JPG"
    obb=uos.listdir()
    print(obb)
    if (lrwFilename not in obb):
        with open(lrwFilename, 'w') as fil:
            fil.write("XXXX")
            print("Created LRW_PIC.JPG")

    lrwFileHandle = open(lrwFilename, 'rb')
    content = lrwFileHandle.read(fileOffset + 32)    #num chars to read
    a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48  = content[fileOffset:fileOffset+32]
    lrwFileHandle.close()

    packetToLRW[17] = a17
    packetToLRW[18] = a18
    packetToLRW[19] = a19
    packetToLRW[20] = a20
    packetToLRW[21] = a21
    packetToLRW[22] = a22
    packetToLRW[23] = a23
    packetToLRW[24] = a24
    packetToLRW[25] = a25
    packetToLRW[26] = a26
    packetToLRW[27] = a27
    packetToLRW[28] = a28
    packetToLRW[29] = a29
    packetToLRW[30] = a30
    packetToLRW[31] = a31
    packetToLRW[32] = a32
    packetToLRW[33] = a33
    packetToLRW[34] = a34
    packetToLRW[35] = a35
    packetToLRW[36] = a36
    packetToLRW[37] = a37
    packetToLRW[38] = a38
    packetToLRW[39] = a39
    packetToLRW[40] = a40
    packetToLRW[41] = a41
    packetToLRW[42] = a42
    packetToLRW[43] = a43
    packetToLRW[44] = a44
    packetToLRW[45] = a45
    packetToLRW[46] = a46
    packetToLRW[47] = a47
    packetToLRW[48] = a48
    crcrcrc=calcCRC(bytearray(packetToLRW),49)
    packetToLRW[49]=crcrcrc>>8
    packetToLRW[50]=crcrcrc&0x00FF
    uart5.write("LRW_PIC OFFSET: " + str(fileOffset) + "\r\n")

def takePic():
    eeData = [0]
    DRAM_PWRC.high()
    CAM_PWRC.high()
    pyb.delay(300)  #don't know if this is needed.
    sensor.reset() # Initialize the camera sensor.
    sensor.set_pixformat(sensor.RGB565) # .GRAYSCALE 8 bits/pix
    sensor.set_framesize(sensor.WQXGA2)   #
    # min: QQQQVGA 40x30  QVGA 320x240  VGA: 640x480
    # WVGA: 720x480  XVGA: 1024x768  SXGA: 1280x1024
    # UXGA: 1600x1200   FHD: 1920x1080  QHD: 2560x1440
    # max: WQXGA2 2592x1944
    pyb.delay(2000) # Let new settings take affect.
    sensor.set_auto_whitebal(True) # Turn off white balance.
    dateTime = rtc.datetime()
    uniqueID = omv.board_id()
    bID = bytearray()  #declares bID as a bytearray
    bID.extend(uniqueID.encode())
    crcrc = calcCRC(bID,24)
    crcStr = (hex(crcrc)).upper()[2:6]
    while (len(crcStr)<4):  #if crc is less than 4 chars pad with leading zeros
        crcStr = '0' + crcStr
    utcDatetimeStr = "xxxx_M_yymmddhhmmss.jpg"
    yearInt=dateTime[0]-2000
    yearStr=str(yearInt)[2:3]
    while (len(yearStr)<2):  #pad with leading zeros
        yearStr = '0' + yearStr
    monthStr=str(dateTime[1])
    while (len(monthStr)<2):  #pad with leading zeros
        monthStr = '0' + monthStr
    dayStr=str(dateTime[2])
    while (len(dayStr)<2):  #pad with leading zeros
        dayStr = '0' + dayStr
    hourStr=str(dateTime[4])
    while (len(hourStr)<2):  #pad with leading zeros
        hourStr = '0' + hourStr
    minStr=str(dateTime[5])
    while (len(minStr)<2):  #pad with leading zeros
        minStr = '0' + minStr
    secStr=str(dateTime[6])
    while (len(secStr)<2):  #pad with leading zeros
        secStr = '0' + secStr
    fullImageFilename = crcStr+"_T_"+yearStr+monthStr+dayStr+hourStr+minStr+secStr+".BMP"
    uart5.write("\r\n"+fullImageFilename+"\r\n")
    EEPROM_SCL = pyb.Pin("D12", pyb.Pin.OUT_PP)
    EEPROM_SDA = pyb.Pin("D13", pyb.Pin.OUT_PP)
    EEDelay = 1
    EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
    EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
    EEPROM_SDA.value(1)
    EEPROM_SCL.value(1)
    eepromSCL_INpull()
    pyb.udelay(100)
    read_bytes_EE(eeData,1,0xDB,0xA2)
    print ('FOLDER LETTER WAS READ AS:', eeData[0])
    if((eeData[0]<FOLDER_LETTER_INIT) or (eeData[0]>FOLDER_LETTER_MAX)):
        EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
        EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
        EEPROM_SDA.value(1)
        EEPROM_SCL.value(1)
        eepromSCL_INpull()
        eepromSDA_INpull()
        pyb.delay(100)
        twi_start_cond_EE_WRITE()
        send_slave_Address_EE(0, 0xA2)
        i2c_write_byte_EE_WRITE(0xFF, 0)
        i2c_write_byte_EE_WRITE(0xDB, 0)
        i2c_write_byte_EE_WRITE(FOLDER_LETTER_INIT, 0)
        twi_stop_cond_EE_WRITE()
        eeData[0]=FOLDER_LETTER_INIT
        print ('SET FOLDER LETTER TO:', FOLDER_LETTER_INIT)
    folderLetterStr = "%1c" % (eeData[0])
    imageFolderName = "FDT_" + crcStr + "_IMG_" + folderLetterStr
    pathFilename = "./" + imageFolderName + "/" + fullImageFilename
    uart5.write(pathFilename+"\r\n")
    obb=uos.listdir()
    if (imageFolderName not in obb):
        uos.mkdir("./" + imageFolderName)
        uart5.write("CREATED NEW FOLDER\r\n")
    count = 0
    lst = uos.listdir(imageFolderName)
    count = len(lst)
    uart5.write("NUM FILES IN IMAGE FOLDER: %d\r\n" % count)
    if (count > MAX_FILES_PER_FOLDER):
        folderLetterStr += 1
        if (folderLetterStr > maxLetter):
            folderLetterStr = initLetter
        EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
        EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
        EEPROM_SDA.value(1)
        EEPROM_SCL.value(1)
        eepromSCL_INpull()
        pyb.udelay(100)
        twi_start_cond_EE_WRITE()
        send_slave_Address_EE(0, 0xA2)
        i2c_write_byte_EE_WRITE(0xFF, 0)
        i2c_write_byte_EE_WRITE(0xDB, 0)
        i2c_write_byte_EE_WRITE(folderLetterStr, 0)
        twi_stop_cond_EE_WRITE()
    obb=uos.listdir("./" + imageFolderName)
    if (fullImageFilename not in obb):
        sensor.snapshot().save(pathFilename)
        uart5.write ("AFTER TOOK PIC\r\n")
        time.sleep_ms(500)
    else:
        uart5.write("SOMETHING BAD ABOUT PATH-FILENAME\r\n")

def minuteFunc(dummy):
    eeReadData=[0]*21
    read_bytes_EE(eeReadData,20,0xE4,0xA2)
    temperature=0
    humidity=0
    barometer=0
    light=0
    uniqueID = omv.board_id()
    bID = bytearray()  #declares bID as a bytearray
    bID.extend(uniqueID.encode())
    crcrc = calcCRC(bID,24)
    crcStr = (hex(crcrc)).upper()[2:6]
#    machine.mem32[WATCHDOG_ADDRESS] = WATCHDOG_KEY
    while (len(crcStr)<4):  #if crc is less than 4 chars pad with leading zeros
        crcStr = '0' + crcStr
    filename_base = crcStr + "_"
    yy,MM,dd,ww,hh,mm,ss,uu = rtc.datetime()
    utcDatetimeStrSpaces = "%2d%2d%2d%2d%2d%2d" % (yy,MM,dd,hh,mm,ss)
    utcDatetimeStr=utcDatetimeStrSpaces.replace(' ', '0')
    localDatetimeStrSpaces = "%2d%2d%2d%2d%2d%2d" % (yy,MM,dd,hh,mm,ss)
    localDatetimeStr=localDatetimeStrSpaces.replace(' ', '0')
    fullFilename = filename_base + utcDatetimeStr[2:8] + ".CSV"    #Use only year-month-day for filename
    dataFolderName = "FDT_" + crcStr + "_DAT"
    pathFilename = "./" + dataFolderName + "/" + fullFilename
    if (DO_WEATHER_SENSORS):
        SENS_PWRC.high() # or p.value(1) to make the pin high (3.3V)
#        if (UART_DEBUGS):
#            uart5.write("MINUTEFUNC BEFORE HUMIDITY\r\n")
        humidity = getHumidity(0x80)
        temperature = getTemperature(0x80)
#        if (UART_DEBUGS):
#            uart5.write("MINUTEFUNC BEFORE ALS\r\n")
        light = getALSfromVEML(0x20)
#        if (UART_DEBUGS):
#            uart5.write("MINUTEFUNC AFTER SENSORS\r\n")
#    SENS_PWRC.low() # or p.value(1) to make the pin high (3.3V)

    obb=uos.listdir()
    if(PRINT_DEBUGS):
        print(obb)
    if (dataFolderName not in obb):
        uos.mkdir("./" + dataFolderName)

    obb=uos.listdir("./" + dataFolderName)
#    if(PRINT_DEBUGS):
#        print(obb)
    if (fullFilename not in obb):
        fileHeader = "RecordType,Date(UTC),Time(UTC),Date(local),Time(local),Temperature,Humidity,AmbLight,Battery,MotDetects,MotPatternHI,MotPatternLO,MotImages,UseAutoRange,DetRangeA(cm),DetRangeB(cm),DetRangeC(cm),DetRangeD(cm),MaxDailyMotImages,Lat,Long\n"
        with open(pathFilename, 'w') as fil:
            fil.write(fileHeader)

#    if (UART_DEBUGS):
#        uart5.write("AFTER FOLDER SEARCH\r\n")
    buildFileLine = "%d" % 1   # Record format version
    buildFileLine += "," + utcDatetimeStr[2:4] + "-" + utcDatetimeStr[4:6] + "-" + utcDatetimeStr[6:8]       # UTC Date
    buildFileLine += "," + utcDatetimeStr[8:10] + ":" + utcDatetimeStr[10:12] + ":" + utcDatetimeStr[12:14]  # UTC Time
    buildFileLine += "," + localDatetimeStr[2:4] + "-" + localDatetimeStr[4:6] + "-" + localDatetimeStr[6:8]       # Local Date
    buildFileLine += "," + localDatetimeStr[8:10] + ":" + localDatetimeStr[10:12] + ":" + localDatetimeStr[12:14]  # Local Time
    buildFileLine += ",%d" % temperature   # Temperature
    buildFileLine += ",%d" % humidity  # Humidity
    buildFileLine += ",%d" % light  # light
    buildFileLine += ",%d" % 0x33  # battery
    buildFileLine += ",%d" % numMotTriggers  # Num motion triggers
    buildFileLine += ",%X" % motPatternHI #motionPattern  # Num motion triggers
    buildFileLine += ",%X" % motPatternLO #motionPattern  # Num motion triggers
    buildFileLine += ",%d" % numMotImages  #
    buildFileLine += ",%d" % SAM_USE_AUTO_LIDAR #
    if((SAM_USE_AUTO_LIDAR & 0x01) == 0x00):
        buildFileLine += ",%d" % SAM_USE_FIXED_MAX # cm
        buildFileLine += ",%d" % SAM_USE_FIXED_MAX  #
        buildFileLine += ",%d" % SAM_USE_FIXED_MAX  #
        buildFileLine += ",%d" % SAM_USE_FIXED_MAX  #
    else:
        buildFileLine += ",%d" % zoneA_max  # cm
        buildFileLine += ",%d" % zoneB_max  #
        buildFileLine += ",%d" % zoneC_max  #
        buildFileLine += ",%d" % zoneD_max  #
    buildFileLine += ",%d," % SAM_MAX_DAILY_MOTION_PICS  #
    buildFileLine += chr(eeReadData[0])  #latSign   # latSign
    buildFileLine += chr(eeReadData[1])  #"%d." % (latWhole)   # latWhole
    buildFileLine += chr(eeReadData[2])
    buildFileLine += chr(eeReadData[3])
    buildFileLine += chr(eeReadData[4])
    buildFileLine += chr(eeReadData[5])
    buildFileLine += chr(eeReadData[6])
    buildFileLine += chr(eeReadData[7])
    buildFileLine += chr(eeReadData[8])
    buildFileLine += ","
    buildFileLine += chr(eeReadData[9])
    buildFileLine += chr(eeReadData[10])
    buildFileLine += chr(eeReadData[11])
    buildFileLine += chr(eeReadData[12])
    buildFileLine += chr(eeReadData[13])
    buildFileLine += chr(eeReadData[14])
    buildFileLine += chr(eeReadData[15])
    buildFileLine += chr(eeReadData[16])
    buildFileLine += chr(eeReadData[17])
    buildFileLine += chr(eeReadData[18])
    buildFileLine += "\n"
#    if(PRINT_DEBUGS):
#        print(buildFileLine)
    with open(pathFilename, 'a') as fil:
        fil.write(buildFileLine)
    if (UART_DEBUGS):
        uart5.write("AFTER CSV WRITE\r\n")
#    pyb.delay(50)
#    if(PRINT_DEBUGS):
#         print("End minuteFunc() %d:%d:%d" % (hh, mm, ss))
#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------
#
#       MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN
#
#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------
PBUTTON_L = pyb.Pin("A0", pyb.Pin.IN)      # A0 Tube Main board 2023
PBUTTON_C = pyb.Pin("C13", pyb.Pin.IN)      # C18 Tube Main board 2023
PBUTTON_R = pyb.Pin("I11", pyb.Pin.IN)      # I11 Tube Main board 2023
WU_BIG = pyb.Pin("A2", pyb.Pin.IN)      # A2 Tube Main board 2023
DRAM_PWRC = pyb.Pin("J10", pyb.Pin.OUT_PP)
DRAM_PWRC.low()
CAM_PWRC = pyb.Pin("D3", pyb.Pin.OUT_PP)
CAM_PWRC.low()
SENS_PWRC = pyb.Pin("K1", pyb.Pin.OUT_PP)
SENS_PWRC.low() # or p.value(0) to make the pin low (0V)
SENS_SCL = pyb.Pin("B10", pyb.Pin.OUT_PP)
SENS_SDA = pyb.Pin("B11", pyb.Pin.OUT_PP)
LCD_SI = pyb.Pin("J11", pyb.Pin.OUT_PP)
LCD_RS = pyb.Pin("E3", pyb.Pin.OUT_PP)
LCD_SCL = pyb.Pin("D11", pyb.Pin.OUT_PP)
LCD_PWRC = pyb.Pin("G9", pyb.Pin.OUT_PP)
LCDoff()    #need long delay (300ms?) between LCDoff() and LCDinit(), otherwise no display of text. Added to beginning of LCDini().
SAM_SDA = pyb.Pin("B0", pyb.Pin.IN)
SAM_SCL = pyb.Pin("G12", pyb.Pin.IN)    #PG12
WU_SAM = pyb.Pin("A3", pyb.Pin.OUT_PP)
WU_SAM.value(1)
LRW_PWRC = pyb.Pin("C3", pyb.Pin.OUT_PP)
IR_FLASH_A = pyb.Pin("E2", pyb.Pin.IN)
IR_FLASH_A.value(1)
IR_FLASH_A.PULL_NONE
IR_FLASH_B = pyb.Pin("K4", pyb.Pin.IN)
IR_FLASH_B.value(1)
IR_FLASH_B.PULL_NONE
LRW_PWRC.value(0)   #LRW Off
LRW_PWRC.value(0)   #LRW Off
FILTER_SW_A = pyb.Pin("G14", pyb.Pin.IN)
FILTER_SW_B = pyb.Pin("PD4", pyb.Pin.IN)
FILTER_SW_A.PULL_NONE
FILTER_SW_B.PULL_NONE
GPS_PWRC = pyb.Pin("K5", pyb.Pin.IN)
GPS_PWRC.PULL_NONE

beginningNewGPSacq = 6
needToResetImageSensor = True
needInitialBaselineImage = True

uart2 = machine.UART(2, 9600, timeout_char = 1000)  #GPS port.
uart5 = machine.UART(5, 9600, timeout_char = 1000)  #DeBug port. both pyb.UART and machine.UART works with oldPy firmware.
uart5.write("BOOTFDT PY ")

rtc = machine.RTC() #machine.RTC: delays pyb.delay works, but datetime does not increment

rtc.datetime((2024, 3, 26, 10, 9, 8, 7, 0))

data=[0]*21
dataDum=[0]

EEPROM_SCL = pyb.Pin("D12", pyb.Pin.OUT_PP)
EEPROM_SDA = pyb.Pin("D13", pyb.Pin.OUT_PP)
EEDelay = 1
EEPROM_SDA.init(EEPROM_SDA.OUT_PP, EEPROM_SDA.PULL_NONE)
EEPROM_SCL.init(EEPROM_SCL.OUT_PP, EEPROM_SCL.PULL_NONE)
EEPROM_SDA.value(1)
EEPROM_SCL.value(1)
eepromSCL_INpull()
pyb.udelay(100)

getGPSwithLCD()

#displayMenu()

read_bytes_EE(data,1,0xC8,0xA2)
if (data[0] == 1):
    uart5.write ("USER PUSHBUTTON\r\n")
    displayMenu()    #need long delay (300ms?) between LCDoff() and LCDinit(), otherwise no display of text. Added to beginning of LCDini().
if (data[0] == 2):
    uart5.write ("MOTION DETECTED\r\n")
if (data[0] == 4):
    uart5.write ("RTC\r\n")
if (data[0] == 8):
    uart5.write ("PWR UP\r\n")

packFromSAM=[0]*64
#print(packFromSAM)
LCDoff()    #Don't put this before LCD, otherwise need 350ms delay between LCDoff() and LCD(init)

#uart5.write ("DISPLAY MENU\r\n")
#displayMenu()
#uart5.write ("END DISPLAY MENU\r\n")

#LCDinit()
#acqGPSwithLCD()

#dateTime = rtc.datetime()
#if ((dateTime[5] == 1) or (dateTime[5] == 31)):
#    if((dateTime[4]>=1) and (dateTime[4]<15)):
#        filterSwitchNight()
#        camFlash(3)
#    else:
#        filterSwitchDay()
#    takePic()
#    camFlash(0)

#askSAMforMotion(packFromSAM)
numMotTriggers = packFromSAM[1]
numMotImages = packFromSAM[2]
numMotImagesTodayPerSAM = (packFromSAM[3] << 8) + packFromSAM[4]
zoneA_max = packFromSAM[5]
zoneB_max = packFromSAM[6]
zoneC_max = packFromSAM[7]
zoneD_max = packFromSAM[8]
motPatternHI = (packFromSAM[16] << 24) + (packFromSAM[15] << 16) + (packFromSAM[14] << 8) + packFromSAM[13]
motPatternLO = (packFromSAM[12] << 24) + (packFromSAM[11] << 16) + (packFromSAM[10] << 8) + packFromSAM[9]

#sendLRW()

#minuteFunc(0)
uart5.write ("AFTER MINUTEFUNC\r\n")

#uart5.write("motPattern: %X %X\r\n" % (motPatternHI, motPatternLO))

pyb.delay(1)  #Need time for LRW xmt. May 13_2024: Works: 50. Might not need much, or any delay, if LRW is before minuteFunc()

uart5.write("MOT FROM SAM: " + str(packFromSAM[1]) + "\r\n")

#dateTime = rtc.datetime()
#uart5.write("\r\n" + str(dateTime[0])+"-"+str(dateTime[1])+"-"+str(dateTime[2])+" "+str(dateTime[4])+":"+str(dateTime[5])+":"+str(dateTime[6])+"\r\n")
#pyb.delay(1200)

#dateTime = rtc.datetime()
#uart5.write(str(dateTime[0])+"-"+str(dateTime[1])+"-"+str(dateTime[2])+" "+str(dateTime[4])+":"+str(dateTime[5])+":"+str(dateTime[6])+"\r\n")
#pyb.delay(1000)


uart5.write("HOLD LEFT BUTTON TO CONNECT TO PC\r\n")
pyb.delay(1000)

if (PBUTTON_L.value() == 0):    #buttons are pulled low, pbutton connects to Vcc.
#    sleepSecs = 60-dateTime[6]
#    uart5.write("SLEEP " + str(sleepSecs) + " SECS\r\n")
    uart5.write("SLEEP\r\n")
    pyb.delay(25)
    DRAM_PWRC.low()
    CAM_PWRC.low()
    # Note the camera will RESET after wakeup from Deepsleep Mode.
#    rtc.wakeup(sleepSecs * 1000)    #set to (0) to turn off wakeup
#    rtc.wakeup(10 * 1000)    #set to (0) to turn off wakeup

    # Enter Deepsleep Mode.
    machine.deepsleep()
    # FDT:  Need to modify firmware /src/micropython/ports/stm32/powerctrl.c powerctl_enter_standby_mode()
    # add re-enabling WKUP1 and WKUP2 with these two lines near the end of the function
    # add re-enabling WKUP1 and WKUP2 with these two lines near the end of the function
    # PWR->WKEPR |= 0x00000203;
    # EXTI->IMR2 |= 0x01800000;
