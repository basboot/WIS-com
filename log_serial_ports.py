from queue import Queue

import serial
import threading

import sys
import subprocess

import time

running = True

def current_milli_time():
    return int(round(time.time() * 1000))

START_TIME = current_milli_time()

serial_queue = Queue(1000)

def serial_read(s):
    while running:
        line = s.readline()
        serial_queue.put("%s,%s" % (current_milli_time()-START_TIME, line.decode("utf-8").rstrip()))
        #print(line)

assert ((sys.platform  == "darwin") or (sys.platform  == "linux") or (sys.platform == "linux2")), "Platform unsupprted"

# executable to find motes
motelist = "./motelist-zolertia"

# alternative version for mac-os
if (sys.platform  == "darwin"):
    motelist = "./motelist-zolertia-macos"

output = (subprocess.Popen([motelist, "-c"], stdout=subprocess.PIPE).communicate()[0]).decode("utf-8")

devices = output.splitlines()

print(devices)

serials = []
threads = []

for device in devices:
    print("Opening serial connection on USB: '%s'" % device.split(",")[1])
    usb = device.split(",")[1]
    ser = serial.Serial(usb, 115200)
    ser.flushInput()
    serials.append(ser)
    threads.append(threading.Thread(target=serial_read, args=(serials[-1],), ))

for thread in threads:
    thread.start()



try:
    while True:
        if (serial_queue.empty()):
            continue
        line = serial_queue.get(True, 1)
        print(line)
except:
    # signal threads to stop
    running = False
    print("An exception occurred")
finally:
    # wait for serial threads to stop
    time.sleep(1)
    for ser in serials:
        ser.close()
