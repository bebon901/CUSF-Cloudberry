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

def read_messages(stream, lock, ubxreader):
    """
    Reads, parses and prints out incoming UBX messages
    """
    # pylint: disable=unused-variable, broad-except

    while reading:
        if stream.in_waiting:
            try:
                lock.acquire()
                (raw_data, parsed_data) = ubxreader.read()
                lock.release()
                if parsed_data:
                    print(parsed_data)
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                continue


def start_thread(stream, lock, ubxreader):
    """
    Start read thread
    """

    thr = Thread(target=read_messages, args=(stream, lock, ubxreader), daemon=True)
    thr.start()
    return thr


def send_message(stream, lock, message):
    """
    Send message to device
    """

    lock.acquire()
    stream.write(message.serialize())
    lock.release()


if __name__ == "__main__":
    port = "/dev/serial0"
    baudrate = 9600
    timeout = 0.1

    with Serial(port, baudrate, timeout=timeout) as serial:
        msg = UBXMessage("CFG", "CFG-NAV5", SET, msgClass=0x06, msgID=0x24, fixMode=2, dyn=1, dynModel=0x06)
        print(msg)
        print(msg.serialize())
        # create UBXReader instance, reading only UBX messages
        ubr = UBXReader(BufferedReader(serial), protfilter=2)

        print("\nStarting read thread...\n")
        reading = True
        serial_lock = Lock()

        read_thread = start_thread(serial, serial_lock, ubr)
        send_message(serial, serial_lock, msg)
        sleep(1)
        msg = UBXMessage("CFG", "CFG-NAV5", POLL)
        send_message(serial, serial_lock, msg)
        sleep(1)
        '''
        print("\nPolling CFG configuration CFG-*...\n")
        for (msgid, msgname) in UBX_MSGIDS.items():
             if msgid[0] == 0x06:  # CFG-* configuration messages
                 msg = UBXMessage("CFG", msgname, POLL)
                 print(msgname, type(msgname))
                 send_message(serial, serial_lock, msg)
                 sleep(1.5)
        '''
