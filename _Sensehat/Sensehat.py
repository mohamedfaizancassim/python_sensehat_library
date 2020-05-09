from smbus2 import SMBus
import struct
import numpy as np
import time


class Sensehat:
    #This class will allow you to access the sensors onboard the 
    def __init__(self):
        #senhat and get their readings.
        self.i2c_channel=1
        #Initialise I2C Device
        self.i2c_bus=SMBus(self.i2c_channel)
        #Define the sensor addresses
        self._LPS25H_address=0x5C
        self._HTS221_address=0x5F
        self._Acceleromter_address=0x1C
        #Initialise the LPS25H sensor for continous reading
        #---------------------------------------------------
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x20,0x00) #Reseting the sensor
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x21,0x84) #Software Reset and Boot
        time.sleep(0.4) #Wait for boot and reset
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x20,0xC6) #PU,BDU,25Hz
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x21,0x40) #FIFO Enable
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x2E,0xC7) #FIFO Mean,Stream:Newest Members, 8 sample moving avg

        #Initialise the HTS221 Sensor for continous reading
        #----------------------------------------------------
        self.i2c_bus.write_byte_data(self._HTS221_address,0x20,0x00) #Reset the sensor
        self.i2c_bus.write_byte_data(self._HTS221_address,0x21,0x80) #Boot
        time.sleep(0.4) #Wait for Boot and Reset
        self.i2c_bus.write_byte_data(self._HTS221_address,0x20,0x87) #PU,BDU and 12.5Hz
        self.i2c_bus.write_byte_data(self._HTS221_address,0x10,0x3F) #AVGT:256 and AVGH:512
        self.i2c_bus.write_byte_data(self._HTS221_address,0x21,0x02) #Enable Heater
        time.sleep(3) #Sleep for 20ms whilst sensor gets heated
        self.i2c_bus.write_byte_data(self._HTS221_address,0x21,0x00) #Disable Heater

    #==========================================
    #    Is Data Ready
    #==========================================
    def IsDataReady(self,sensor_address,temp):
        status=self.i2c_bus.read_byte_data(sensor_address,0x27)
        status=status&0x03 #Only interested in the first two bytes
        if(status!=0x00):
            if(temp==True):
                if (status==0x01 or status==0x03):
                    return True
                else:
                    return False
            else:
                return True
        else:
            return False

    def Get_Reference_Pressure(self):
        #Getting Reference Pressue
        ref_press_xl=self.i2c_bus.read_byte_data(self._LPS25H_address,0x08)
        ref_press_l=self.i2c_bus.read_byte_data(self._LPS25H_address,0x09)
        ref_press_h=self.i2c_bus.read_byte_data(self._LPS25H_address,0x0A)

        #Contatonating Ref Pressuer bytes
        ref_press=bytearray([0x00,ref_press_h,ref_press_l,ref_press_xl])
        #2's Complement Conversion
        ref_press_i=struct.unpack('>i',ref_press)[0]
        #ref_press_i=twos_complement_toInt(ref_press_i)
        return ref_press_i

    def Set_Reference_Pressure(self,ref_pressure_hpa):
        #Converting Reference Pressure to 2's Compliment 24-bit value
        ones_c=~ref_pressure_hpa
        twos_c=ones_c+1
        bin_val=0
        if(ref_pressure_hpa<0):
            bin_val=twos_c|(1<<ref_pressure_hpa.bit_length())
        else:
            bin_val=twos_c
        #Split bin val into 3 bytes
        ref_press_xl=bin_val&0xFF
        ref_press_l=(bin_val&0xFF00)>>8
        ref_press_h=(bin_val&0xFF0000)>>16

        #Write byte values to relevant registers
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x08,ref_press_xl)
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x09,ref_press_l)
        self.i2c_bus.write_byte_data(self._LPS25H_address,0x0A,ref_press_h)

    def Read_Temperature_C_LPS25H(self):
        while True:
            if (self.IsDataReady(self._HTS221_address,True)):
                break
        #Read temp_l and temp_h registers
        temp_l=self.i2c_bus.read_byte_data(self._LPS25H_address,0x2B)
        temp_h=self.i2c_bus.read_byte_data(self._LPS25H_address,0x2C)
        #Debug
        #print("temp_h:{} temp_l:{} .".format(bin(temp_h),bin(temp_l)))
        #Concatonating temp_l and temp_H
        temp=bytearray([temp_h,temp_l])
        #print("Temperature Byte Value: {} {} ".format(temp_h,temp_l))
        #Converting to short
        temp_i=struct.unpack('>h',temp)[0]
        #temp_i=twos_complement_toInt(temp_i)
        print("Temperature | Raw Bin: {}{} 2's Complement Bin: {}".format(bin(temp_h),bin(temp_l),bin(temp_i)))
        temperature = (42.5+(temp_i/480))
        return round(temperature,2)

    def Read_Temperature_C_HTS221(self):
        while True:
            if (self.IsDataReady(self._HTS221_address,True)):
                break
        #Obtaining t0x8 and t1x8
        t0_degC_x8=self.i2c_bus.read_byte_data(self._HTS221_address,0x32)
        t1_degC_x8=self.i2c_bus.read_byte_data(self._HTS221_address,0x33)
        #Obtaining t0/t1 msb
        t0t1_msb=self.i2c_bus.read_byte_data(self._HTS221_address,0x35)
        t0_msb=t0t1_msb&0x03 #Masking t0 bits
        t1_msb=t0t1_msb&0x0C #Getting t1 MSB
        t1_msb=t1_msb>>2 #Right shift by 2
        #Concatonating
        t0_degC_x8=bytearray([t0_msb,t0_degC_x8])
        t1_degC_x8=bytearray([t1_msb,t1_degC_x8])
        t0_degC_us=(struct.unpack('>H',t0_degC_x8)[0]/8) #Convert as unsigned short
        t1_degC_us=(struct.unpack('>H',t1_degC_x8)[0]/8)
        #print("Binary T0 Out: {}, T1 Out: {}".format(bin(t0_degC_us),bin(t1_degC_us)))
        #Obtaining t0_out
        t0_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x3C)
        t0_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x3D)
        t0=bytearray([t0_h,t0_l])
        t0_s=struct.unpack('>h',t0)[0]
        #t0_s=twos_complement_toInt(t0_s)
        #Obtaining t1_out
        t1_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x3E)
        t1_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x3F)
        t1=bytearray([t1_h,t1_l])
        t1_s=struct.unpack('>h',t1)[0]
        #t1_s=twos_complement_toInt(t1_s)
        #print("T0_DegC:{} , T1_DegC:{}".format(t0_s,t1_s))
        #Reading t_out
        tOut_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x2A)
        tOut_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x2B)
        tOut=bytearray([tOut_h,tOut_l])
        tOut_s=struct.unpack('>h',tOut)[0]
        #tOut_s=twos_complement_toInt(tOut_s)
        #Calculation
        temperature_degC=((t1_degC_us-t0_degC_us)*(tOut_s-t0_s)/(t1_s-t0_s))+t0_degC_us
        return round(temperature_degC,1)
   
    def Read_Pressure_HPa(self):
        while True:
            if (self.IsDataReady(self._LPS25H_address,False)):
                break
        #Read press_xl,press_l and press_h registers
        press_xl=self.i2c_bus.read_byte_data(self._LPS25H_address,0x28)
        press_l=self.i2c_bus.read_byte_data(self._LPS25H_address,0x29)
        press_h=self.i2c_bus.read_byte_data(self._LPS25H_address,0x2A)
        #Debug
        #print("press_h:{} press_l:{} press_xl:{} .".format(bin(press_h),bin(press_l),bin(press_xl)))
        #Concatonating 
        press=bytearray([0x00,press_h,press_l,press_xl])
        #Converting to int32
        press_i=(struct.unpack('>i',press)[0])
        print("Pressure | Raw Bin: {}{}{} 2's Complement Bin: {}".format(bin(press_h),bin(press_l),bin(press_xl),bin(press_i)))
        #press_i=twos_complement_toInt(press_i)
        #Calculation
        pressure= (press_i/4096)
        return round(pressure,2)

    def Read_Humidity_Perc(self):
        while True:
            if (self.IsDataReady(self._HTS221_address,False)):
                break
        #Read h0 and h1 registers 
        h0=self.i2c_bus.read_byte_data(self._HTS221_address,0x30)
        h1=self.i2c_bus.read_byte_data(self._HTS221_address,0x31)
        #Read h0t0 registers
        h0t0_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x36)
        h0t0_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x37)
        #Read t1 registers
        h1t0_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x3A)
        h1t0_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x3B)
        #Read raw_humidity registers
        rawHum_l=self.i2c_bus.read_byte_data(self._HTS221_address,0x28)
        rawHum_h=self.i2c_bus.read_byte_data(self._HTS221_address,0x29)

        #Converting h0 and h1 to short
        h0_us=(struct.unpack('>H',bytearray([0x00,h0]))[0]/2)
        h1_us=(struct.unpack('>H',bytearray([0x00,h1]))[0]/2)
        #Converting h0t0 to short
        h0t0=bytearray([h0t0_h,h0t0_l])
        h0t0_s=struct.unpack('>h',h0t0)[0]
        #h0t0_s=twos_complement_toInt(h0t0_s)
        #Converting t0 to short
        h1t0=bytearray([h1t0_h,h1t0_l])
        h1t0_s=struct.unpack('>h',h1t0)[0]
        #h1t0_s=twos_complement_toInt(h1t0_s)
        #Converting raw_hum to short
        rawHum=bytearray([rawHum_h,rawHum_l])
        rawHum_s=struct.unpack('>h',rawHum)[0]
        #rawHum_s=twos_complement_toInt(rawHum_s)

        #Calculation
        humidity=((h1_us-h0_us)*(rawHum_s-h0t0_s)/(h1t0_s-h0t0_s))+h0_us
        return round(humidity,2)



     

