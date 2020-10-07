# Very simple Python example to check serial communication with the WIS
import time

import serial

if __name__ == '__main__':

    # TODO: Check timeout
    # TODO: the FF can probably run higher baudrates
    ser = serial.Serial('/dev/cu.SLAB_USBtoUART3', 115200, timeout=0.050)

    deg = 0
    while True:
        # check for serial input
        if ser.in_waiting > 0:
            # read incoming serial messsage until endline
            message = ser.readline().decode("utf-8")

            # split to decode message
            data = message.split(" ")

            # Is firs char not an "S" then we do not have a sensor reading, so ignore the message
            if (data[0] != "S"):
                print("WARNING: did not understand: %s" % message)
            else:
                # decode the message
                print("Node %d Sensor1: %d Sensor2: %d" % (int(data[1]), int(data[2]), int(data[3])))

            # spin servo each time a message has been received
            if deg == 0:
                deg = 180
            else:
                deg = 0

            node = 247

            # create command from node_id with actuator and degrees for servo
            command = "%d %d\n" % (node, deg)

            # write command over the serial line
            ser.write(command.encode('utf-8'))

        time.sleep(0.1)

    ser.close()  # close port
