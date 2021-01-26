from queue import Queue

import serial
import threading

import sys
import subprocess

import time

# to filter out the serial data we want to log
# sensors_to_log = [247, 210]
# sensors_data = {247: {'s1': 0, 's2': 0, 'a': 0}, 210: {'s1': 0, 's2': 0, 'a': 0}}
# sensor_to_sync = 247

sensors_to_log = [201, 202, 203, 204]
sensors_data = {201: {'s1': 0, 's2': 0, 'a': 0}, 202: {'s1': 0, 's2': 0, 'a': 0}, 203: {'s1': 0, 's2': 0, 'a': 0}, 204: {'s1': 0, 's2': 0, 'a': 0}}
sensor_to_sync = 204

ports_to_skip = ['/dev/tty.SLAB_USBtoUART3', '/dev/tty.SLAB_USBtoUART4', '/dev/tty.SLAB_USBtoUART6', '/dev/tty.SLAB_USBtoUART9']
# to stop the threads
running = True

# current time in milliseconds
def current_milli_time():
    return int(round(time.time() * 1000))

# to calculate time relative to start of script
START_TIME = current_milli_time()

# queue to keep all messages
serial_queue = Queue(1000)

def log_sensors():
    data = ""
    for sensor in sensors_to_log:
        data = data + "%s,%s,%s,%s," % (sensor, sensors_data[sensor]["s1"], sensors_data[sensor]["s2"], sensors_data[sensor]["a"])

    print("%s,%s" % (current_milli_time()-START_TIME, data))

def serial_read(s):
    while running:
        line = s.readline()

        data = line.decode("utf-8").rstrip().split(',')

        # skip error messaged
        if (len(data) > 1):
            sensor = int(data[1])
            if (sensor in sensors_data.keys()):
                # python dict is thread safe https://docs.python.org/3/glossary.html#term-global-interpreter-lock
                sensors_data[sensor]['s1'] = int(data[2])
                sensors_data[sensor]['s2'] = int(data[3])
                sensors_data[sensor]['a'] = int(data[6])

                if (sensor == sensor_to_sync):
                    log_sensors()

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

#devices = ['/dev/cu.SLAB_USBtoUART', '/dev/cu.SLAB_USBtoUART6']

for device in devices:
    if device.split(",")[1] in ports_to_skip:
        continue
    print("Opening serial connection on USB: '%s'" % device.split(",")[1])
    usb = device.split(",")[1]
    ser = serial.Serial(usb, 460800)
    ser.flushInput()
    serials.append(ser)
    threads.append(threading.Thread(target=serial_read, args=(serials[-1],), ))

for thread in threads:
    thread.start()



try:
    while True:
        # if (serial_queue.empty()):
        #     continue
        # line = serial_queue.get(True, 1)
        # print(line)
        pass
except:
    # signal threads to stop
    running = False
    print("An exception occurred")
finally:
    # wait for serial threads to stop
    time.sleep(1)
    for ser in serials:
        ser.close()
