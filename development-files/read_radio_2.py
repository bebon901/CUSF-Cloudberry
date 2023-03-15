"""Monitors the UART port on a Raspberry Pi 3 for Spektrum serial packets
Assumes the packets follow the Remote Receiver format
Forwards the packets on the TX pin of the serial port, so you can pass the
packets on the the flight control board
"""
import serial
import time
import sys

def align_serial(ser):
    """Aligns the serial stream with the incoming Spektrum packets
    Spektrum Remote Receivers (AKA Spektrum Satellite) communicate serially
    in 16 byte packets at 125000 bits per second (bps)(aka baud) but are
    compatible with the standard 115200bps rate. We don't control the output
    transmission timing of the Spektrum receiver unit and so might start
    reading from the serial port in the middle of a packet transmission.
    To align the reading from the serial port with the packet transmission,
    we use the timing between packets to detect the interval between packets
    Packets are communicated every 11ms. At 115200 bps, a bit is read in 
    approximately 8.69us, so a 16 byte (128 bit)
    packet will take around 1.11ms to be communicated, leaving a gap of about
    9.89ms between packets. We align our serial port reading with the protocol
    by detecting this gap between reads.
    Note that we do not use the packet header contents because
        1) They are product dependent. Specifically, "internal" Spektrum
        receivers indicate the system protocol in the second byte of the header
        but "external" receivers do not. Further, different products are
        use different protocols and indicate this using the
        system protocol byte.
        2) Other bytes in the packet may take on the same value as the header
        contents. No bit patterns of a byte are reserved, so any byte in the
        data payload of the packet could match the values of the header bytes.
    Inputs
    ------
    ser: serial.Serial instance
        serial port to read from
    """
    data = None
    # read in the first byte, might be a long delay in case the transmitter is
    # off when the program begins
    ser.read(1)
    dt = 0
    # wait for the next long delay between reads
    dt_threshold = 0.005 # pick some threshold between 8.69us and 9.89ms
    while dt < dt_threshold:
        start = time.time()
        ser.read()
        dt = time.time()-start
    # consume the rest of the packet
    ser.read(15)
    # should be aligned with protocol now

MASK_CH_ID = 0b01111000 # 0x78
SHIFT_CH_ID = 3
MASK_SERVO_POS_HIGH = 0b00000111 # 0x07
def parse_packet(packet):
            phase = packet[0]
            channel = int(packet[1:5], 2)
            value = int(packet[5:16], 2)
            return phase, channel, value
values = {0:["Throttle", 0], 1:["Aileron", 0], 2:["Elevator", 0], 3:["Rudder", 0], 4:["Aux", 0], 5:["CH5", 0], 6:["CH6", 0], 7:["CH7", 0], 8:["CH7", 0], 9:["CH7", 0], 10:["CH7", 0], 11:["CH7", 0], 12:["CH7", 0], 13:["CH7", 0], 14:["CH7", 0], 15:["CH7", 0], 16:["CH7", 0]}
def parse_bytes(bytess):
    global values
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
                values[subd[1]] = [values[subd[1]][0], subd[2]]
            #print(packet[i])
        return values
    else:
        return None

def parse_channel_data(data):
    """Parse a channel's 2 bytes of data in a remote receiver packet
    Inputs
    ------
    data: 2 byte long string (currently only supporting Python 2)
        Bytes within the remote receiver packet representing a channel's data
    Outputs
    -------
    channel_id, channel_data
    """
    ch_id = (ord(data[0]) & MASK_CH_ID) >> SHIFT_CH_ID
    ch_data = (
        ((ord(data[0]) & MASK_SERVO_POS_HIGH) << 8) | ord(data[1]))
    ch_data = 988 + (ch_data >> 1)
    return ch_id, ch_data

print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")
ser = serial.Serial(
    port="/dev/serial0", baudrate=125000,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE)
N_CHAN = 13
data = None
servo_position = [0 for i in range(N_CHAN)]
try:
    align_serial(ser)
    while True:
        data_buf = ser.read(16)
        data = data_buf
        #print(data)
        #for i in range(7):
        #    print(data[2*i:2*i+2])
        #    #ch_id, s_pos = parse_channel_data(data[2*i:2*i+2])
        #    #servo_position[ch_id] = s_pos
        sys.stdout.write(
            "    %4d     %4d     %4d     %4d     %4d     %4d\r" %tuple((parse_bytes(data)[0][1], parse_bytes(data)[1][1],parse_bytes(data)[2][1],parse_bytes(data)[3][1],parse_bytes(data)[4][1],parse_bytes(data)[5][1])))
        sys.stdout.flush()
        ser.write(data_buf)
except(KeyboardInterrupt, SystemExit):
    ser.close()
except(Exception) as ex:
    print(ex)
    ser.close()
