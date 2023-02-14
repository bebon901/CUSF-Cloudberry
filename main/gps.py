from sys import platform
from io import BufferedReader
from threading import Thread, Lock
from time import sleep
from serial import Serial
from pyubx2 import (
    UBXMessage,
    UBXReader,
    POLL,
    SET,
    UBX_MSGIDS,
)

# initialise global variables
reading = False
latest_reading = ""
prev_read = ""
def read_messages(stream, lock, ubxreader):
    global latest_reading, prev_read
    while reading:
        if stream.in_waiting:
            try:
                lock.acquire()
                (raw_data, parsed_data) = ubxreader.read()
                lock.release()
                if parsed_data:
                    print(parsed_data)
                    prev_read = latest_reading
                    latest_reading = parsed_data
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                continue


def start_thread(stream, lock, ubxreader):
    thr = Thread(target=read_messages, args=(stream, lock, ubxreader), daemon=True)
    thr.start()
    return thr

def send_message(stream, lock, message):
    lock.acquire()
    stream.write(message.serialize())
    lock.release()
port = "/dev/serial0"
baudrate = 9600
timeout = 0.1
serial_lock = Lock()
#serial = Serial(port, baudrate, timeout=timeout)
def gps_mode():
      model = 0
      serial = Serial(port, baudrate, timeout=timeout)
      global prev_read, serial_lock
      #try:
      msg = UBXMessage("CFG", "CFG-NAV5", SET, msgClass=0x06, msgID=0x24, fixMode=2, dyn=1, dynModel=0x06)
      print(msg)
      print(msg.serialize())
      # create UBXReader instance, reading only UBX messages
      ubr = UBXReader(BufferedReader(serial), protfilter=2)

      print("\nStarting read thread...\n")
      reading = True
      #serial_lock = Lock()
      read_thread = start_thread(serial, serial_lock, ubr)
      sleep(1)
      while model != 6:
        #try:
        send_message(serial, serial_lock, msg)
        sleep(1)
        msg = UBXMessage("CFG", "CFG-NAV5", POLL)
        send_message(serial, serial_lock, msg)
        sleep(1)
        print(prev_read)
        model = int(str(prev_read).split(",")[11].replace(" dynModel=", ""))
        print("Model is:", str(model))
        #except:
        #  print("Something Didn\'t work that time, I\'ll try again")
      print("GPS Successfully in Flight Mode!!")


#start_thread()
#put_in_airborne_mode()
gps_mode()
