"""Monitors the UART port on a Raspberry Pi 3 for Spektrum serial packets
Assumes the packets follow the Remote Receiver format
Forwards the packets on the TX pin of the serial port, so you can pass the
packets on the the flight control board
"""
import serial
import time

MASK_CH_ID = 0b01111000 # 0x78
SHIFT_CH_ID = 3
MASK_SERVO_POS_HIGH = 0b00000111 # 0x07

def parse_packet(packet):
                phase = packet[0]
                channel = int(packet[1:5], 2)
                value = int(packet[5:16], 2)
                return phase, channel, value
valuess = {0:["Throttle", 0], 1:["Roll", 1024], 2:["Pitch", 1024], 3:["Yaw", 0], 4:["AUX1", 0], 5:["AUX2", 0], 6:["CH6", 0], 7:["CH7", 0], 8:["CH7", 0], 9:["CH7", 0], 10:["CH7", 0], 11:["CH7", 0], 12:["CH7", 0], 13:["CH7", 0], 14:["CH7", 0], 15:["CH7", 0], 16:["CH7", 0]}
output = {"Throttle": 406, "Pitch":1024, "Roll":1024, "Yaw":0, "AUX1":0, "AUX2":0}

def parse_bytes(bytess):
        global values, output
        data = bytess
        hexdat= data.hex()
        #print(bytess)
        #print(hexdat)
        #print(len(hexdat))
        packet = []
        if len(hexdat) == 32:
            for i in range(0, 16, 2):
                subp = str(bin(data[i])).replace("0b", "")
                #print(data[i])
                #print(subp)
                while len(subp)< 8:
                    subp = "0" + subp
                #print(subp)
                subp2 = str(bin(data[i+1])).replace("0b", "")
                while len(subp2)< 8:
                    subp2 = "0" + subp2
                #print(subp2)
                packet.append(subp + subp2)
            #print(packet)
            
            #values = {0:["Throttle", 0], 1:["Aileron", 0], 2:["Elevator", 0], 3:["Rudder", 0], 4:["Aux", 0], 5:["CH5", 0], 6:["CH6", 0], 7:["CH7", 0], 8:["CH7", 0], 9:["CH7", 0], 10:["CH7", 0], 11:["CH7", 0], 12:["CH7", 0], 13:["CH7", 0], 14:["CH7", 0], 15:["CH7", 0], 16:["CH7", 0]}
            for i in range(len(packet)):
                subd = parse_packet(packet[i])
                if (subd[2] != 1024) and (subd[2] != 0):
                  if subd[2] > 405:
                    valuess[subd[1]] = [valuess[subd[1]][0], subd[2]]
                #print(packet[i])
            for i in range(len(valuess)):
                try:
                    output[valuess[i][0]] = valuess[i][1]
                except:
                    pass
            return output
        else:
            return None


class values:
    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.yaw = 0
        self.pitch = 0
        self.aux1 = 0
        self.aux2 = 0
        self._data = ""
        self.serialport = "/dev/serial0"
 
    @property
    def data(self):
        return self._data
    
    @data.setter
    def data(self, value):
        self._data = value
        dat = parse_bytes(value)
        self._throttle = dat[0][1]
        self.roll = dat[1][1]
        self.yaw = dat[3][1]
        self.pitch = dat[2][1]
        self.aux1 = dat[4][1]
        self.aux2 = dat[5][1]
        print(self.throttle)
    @property
    def throttle(self):
        return self._throttle
    @throttle.setter
    def throttle(self, value):
        self._throttle = value


