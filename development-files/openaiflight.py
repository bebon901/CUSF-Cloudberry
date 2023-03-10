import serial

# Open the serial port
ser = serial.Serial("/dev/serial0", 9600, timeout=1)

# Define the flight mode command
u5 = bytearray([
0xB5,0x62,0x06,0x24,
0x24,0x00,
0x06,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x55,0xB4
])
u6 = bytearray([0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC])
ubx5 = bytearray([
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, # Header/Command/Size
  0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, # Payload data (Dynamics flag, and Dynamic setting)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x55, 0xB4 
])
# Send the flight mode command to the GPS module
ser.write(ubx5)

# Read the response from the GPS module
for i in range(3):
    response = ser.read(20)
    #print(response)
    ser.write(ubx5)
    # Check the response for the expected ACK (b'\xb5\x62\x06\x01\x02\x00\x01\x06\x00\xd5\x0d')
    #if response == b'\xb5\x62\x06\x01\x02\x00\x01\x06\x00\xd5\x0d':
    #    print("GPS is in flight mode")
    #else:
    #    print("Failed to enter flight mode")

# send the UBX message to retrieve the navigation configuration
ser.write(b'\xb5\x62\x06\x24\x00\x00\x2a\x84')
for i in range(20):
   # read the response from the GPS
   response = ser.read(60)
   print(response)
   # parse the response to find the dynamics configuration
   dynamics_config = response[10]

   # c  heck if the GPS is in airborne mode (dynamics_config == 6)
   if dynamics_config == 6:
      print("The GPS is in airborne mode")
   else:
      print("The GPS is not in airborne mode")

# close the serial port
ser.close()
# Close the serial port
ser.close()
