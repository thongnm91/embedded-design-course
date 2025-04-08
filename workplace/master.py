#lib
import json
import sys
import time
import random
import serial
from iotticket.models import device
from iotticket.models import criteria
from iotticket.models import deviceattribute
from iotticket.models import datanodesvalue
from iotticket.client import Client

port = serial.Serial('COM26', baudrate = 9600, timeout=2, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE);
global received;
port.close();

#main
# data = json.load(open("config.json"))
# username = data["username"]
# password = data["password"]
# deviceId = data["deviceId"]
# baseurl = data["baseurl"]

# c = Client(baseurl, username, password)

def send_data_to_iot_ticket(value,value2):
    print("WRITE DEVICE DATANODES FUNTION.")
    listofvalues=[]
    nv = datanodesvalue()
    nv.set_name("Humidity") #needed for writing datanode
    nv.set_path("hum")
    nv.set_dataType("double")
    nv.set_unit("%")
    nv.set_value(value) #needed for writing datanode
    nv1 = datanodesvalue()
    nv1.set_name("Temperature") #needed for writing datanode
    nv1.set_path("temp")
    nv1.set_dataType("double")
    nv1.set_unit("C")
    nv1.set_value(value2) #needed for writing datanode
    listofvalues.append(nv)
    listofvalues.append(nv1)
    print(c.writedata(deviceId, *listofvalues)) # another way to make this would be c.writedata(deviceId, nv, nv1)
    print("END WRITE DEVICE DATANODES FUNCTION")
    print("-------------------------------------------------------\n")

def modbus_request():
    #--------------------------------------------------------------------#
    hexadecimal_string = [0x01,0x04,0x00,0x01,0x00,0x01,0x60,0x0A];
        #01 04 00 01 00 01 60 0A
        #https://rapidscada.net/modbus/
        #04 read input registers, 01 sensor 1, 01 number of frame reply 1, 60 0A check sum. 
    #--------------------------------------------------------------------#
    
    
    print("request",hexadecimal_string);
    port.open();
    port.write(hexadecimal_string);
    received=port.read(8);
    if len(received) > 5:
        print("{}: respond: {}".format(time.strftime('%Y%m%d:%H%M%S'), received))
        value=((received[3]<<8)|received[4])&0x0FFF   #adc value to real value
        print("")
    else:
        value = -9999
        if len(received) == 0:
            print("{}: Empty response, no data".format(time.strftime('%Y%m%d:%H%M%S')))
            print("")            
        else:
            print("{}: Received broken data frame: {}".format(time.strftime('%Y%m%d:%H%M%S'), received))
    time.sleep(1);
    port.close();
    return value;

while True:
    #write datanode demo
    #value=random.uniform(70, 100); #humidity value
    #value2=random.uniform(50, 100);
    value=modbus_request()/10.0; #read temperature and divided by 10 (*-1 for demo)
    print(value,"Celsius")
    #send_data_to_iot_ticket(value,value2); 
    time.sleep(3);
