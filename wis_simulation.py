#!/usr/bin/python

# Jim Paris <jim@jtan.com>

# Simple terminal program for serial devices.  Supports setting
# baudrates and simple LF->CRLF mapping on input, and basic
# flow control, but nothing fancy.

# ^C quits.  There is no escaping, so you can't currently send this
# character to the remote host.  Piping input or output should work.

# Supports multiple serial devices simultaneously.  When using more
# than one, each device's output is in a different color.  Input
# is directed to the first device, or can be sent to all devices
# with --all.

import sys
import os
import serial
import threading
import traceback
import time
import signal
import fcntl
import string
import re
import numpy as np
import subprocess

# to filter out the serial data we want to log
sensors_to_log = [201, 202, 203, 204]
sensors_data = {201: {'s1': 0, 's2': 0, 'a': 0}, 202: {'s1': 0, 's2': 0, 'a': 0}, 203: {'s1': 0, 's2': 0, 'a': 0}, 204: {'s1': 0, 's2': 0, 'a': 0}}
sensor_to_sync = 204

# manually add port to log here
files_to_log = {"SLAB_USBtoUART": {"filename" : "tmp_sensorlog1.txt", "file": None, "lines": None},
                "SLAB_USBtoUART4": {"filename" : "tmp_sensorlog2.txt", "file": None, "lines": None},
                "SLAB_USBtoUART6": {"filename" : "tmp_sensorlog3.txt", "file": None, "lines": None},
                "SLAB_USBtoUART8": {"filename" : "tmp_sensorlog4.txt", "file": None, "lines": None}}

# manually add ports to skip here
ports_to_skip = ['/dev/tty.SLAB_USBtoUART10']

# to stop the threads
running = True

# current time in milliseconds
def current_milli_time():
    return int(round(time.time() * 1000))

f = None


class WisSimulation:
    def __init__(self):
        self.current_epoch = -1
        self.f_started = False
        self.f_received = 0
        self.r_received = 0
        self.s_received = 0

        self.radios = [0, 0, 0, 0]
        self.flows = [0, 0, 0, 0]
        self.sleeps = [0, 0, 0, 0]

        self.last_sleep = 1;

        self.pressures = np.array([[0, 0, 0, 0, 0, 0, 0]])
        self.simulationHasRun = False

        self.wisA = np.array([[0.0187827220639132, 0.00544358753949081, 0.0180884502118929, 0.0184024705120851, 0.0186935030568089, 0.00542638981803773, 0.00542708276554741]])
        self.wisB = np.array([[-188.021315006585, -65.1941797690961, -181.441732948265, -182.897742210961, -184.083395912647, -59.3130155705813, -59.6096693397852]])

        # Discrete plant model copied from, HIL (zoh)
        self.SPS = 1
        self.Apd = np.array([[1, 0.529741518927619, 0, 0, 0, 0], [0, 0.214711172341697, 0, 0, 0, 0], [0, 0, 1, 0.992594124612871, 0, 0], [0, 0, 0, 0.0574326192676173, 0, 0], [0, 0, 0, 0, 1, 0.403906791292383], [0, 0, 0, 0, 0, 0.263597138115727]])
        self.Bpd = np.array([[-0.00187762870622354, -0.0899442345745638, 0], [0.136116730127439, 0, 0], [0, 0.0477678788945988, -0.1404099971918], [0, 0.0879729555350224, 0], [0, 0, -0.00764986783870191], [0, 0, 0.147280572376855]])
        self.Cpd = np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0]])
        self.Dpd = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

        self.Epd = -0.015 / 0.2279 * 1 / 60 * 1 / self.SPS *  np.array([[0], [0], [0], [0], [1], [0]])

        self.xpd = np.array([[0.25], [0], [0.20], [0], [0.15], [0]]) # digital model does not have the disturbance state

        self.kk = 0

    def handleMessage(self, command, id, epoch, data, serial):
        if command == "f":
            #print("flows received")
            # Sanity check first
            if not self.f_started:
                self.f_started = True
                if epoch != self.current_epoch + self.last_sleep:
                    #assert self.current_epoch == -1, "New epoch is not the last + sleeping periods"
                    print("WARNING: did we mis a period????") # created a warning because when sleeping this might happen
                print("#### %d ####" % epoch)
                self.current_epoch = epoch
            assert self.current_epoch == epoch, "Received f for wrong epoch"
            self.f_received += 1
            if self.r_received == 4:
                self.r_received = 0
            assert self.r_received == 0, "Did not receive all r"

            # store flow
            flow = data/1000000
            print(flow)
            self.flows[id - 201] = flow

            # reset to activate next sim
            self.simulationHasRun = False

        if command == "s":
            #print("sleep received")
            # Sanity check first

            if self.current_epoch > -1:
                assert self.current_epoch == epoch, "Received s for wrong epoch"
            self.s_received += 1


            # store sleep
            self.sleeps[id - 201] = data

            # process all received radio's
            # TODO: later is sim works

        if command == "r":
            #print("radio received")
            # Sanity check first
            # Are we in the correct epoch
            if self.current_epoch > -1:
                assert self.current_epoch == epoch, "Received r for wrong epoch"
            # count r
            self.r_received += 1
            # Did we receive all s before r starts? (could go wrong if we did not receive any)
            if self.s_received == 4:
                self.s_received = 0
            assert self.s_received == 0, "Did not receive all s"
            self.f_started = False
            if self.f_received == 4:
                self.f_received = 0
            assert self.f_received == 0, "Did not receive all f"

            # store radio
            self.radios[id - 201] = data

            # update simulation, using received sleep
            # only process once
            if not self.simulationHasRun:
                print("sim")
                # avoid double update
                self.simulationHasRun = True

                # run sim
                u = np.array([[self.flows[0]], [self.flows[1]], [self.flows[2]]])

                self.last_sleep = self.sleeps[id-201]
                for i in range(self.sleeps[id-201]): # use sleep time from the node that triggered the update
                    for j in range(self.SPS):
                        self.xpd = np.add(np.add(np.matmul(self.Apd, self.xpd), np.matmul(self.Bpd, u)),  self.Epd * (1 if self.kk >= 20 else 0))
                        self.kk += 1

                yd = np.matmul(self.Cpd, self.xpd)
                print(yd)

                # convert water levels to pressure
                levels = np.array([[0.30, yd[0], yd[0], yd[1], yd[1], yd[2], yd[2]]])
                self.pressures = np.divide((np.subtract((levels * 100),  self.wisB)), self.wisA)

            # send water levels
            pressures = "%d %d %d\n" % (id, self.pressures[0,(id - 201)*2], 0 if id == 204 else self.pressures[0,(id - 201)*2 + 1])
            # print(pressures)
            serial.write(pressures.encode())

def firefly_devices():

    assert ((sys.platform == "darwin") or (sys.platform == "linux") or (
                sys.platform == "linux2")), "Platform unsupprted"

    # executable to find motes
    motelist = "./motelist-zolertia"

    # alternative version for mac-os
    if (sys.platform == "darwin"):
        motelist = "./motelist-zolertia-macos"

    output = (subprocess.Popen([motelist, "-c"], stdout=subprocess.PIPE).communicate()[0]).decode("utf-8")

    devices = output.splitlines()

    print(devices)

    serials = []
    threads = []

    # devices = ['/dev/cu.SLAB_USBtoUART', '/dev/cu.SLAB_USBtoUART6']

    ffs = []

    for device in devices:
        if device.split(",")[1] in ports_to_skip:
            continue
        ffs.append(device.split(",")[1])

    return ffs

# Need OS-specific method for getting keyboard input.
if os.name == 'nt':
    import msvcrt
    class Console:
        def __init__(self, bufsize = 1):
            # Buffer size > 1 not supported on Windows
            self.tty = True
        def cleanup(self):
            pass
        def getkey(self):
            start = time.time()
            while True:
                z = msvcrt.getch()
                if z == '\0' or z == '\xe0': # function keys
                    msvcrt.getch()
                else:
                    if z == '\r':
                        return '\n'
                    return z
                if (time.time() - start) > 0.1:
                    return None

    class MySerial(serial.Serial):
        def nonblocking_read(self, size=1):
            # Buffer size > 1 not supported on Windows
            return self.read(1)
elif os.name == 'posix':
    import termios, select, errno
    class Console:
        def __init__(self, bufsize = 65536):
            self.bufsize = bufsize
            self.fd = sys.stdin.fileno()
            if os.isatty(self.fd):
                self.tty = True
                self.old = termios.tcgetattr(self.fd)
                tc = termios.tcgetattr(self.fd)
                tc[3] = tc[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
                tc[6][termios.VMIN] = 1
                tc[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSANOW, tc)
            else:
                self.tty = False
        def cleanup(self):
            if self.tty:
                termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)
        def getkey(self):
            # Return -1 if we don't get input in 0.1 seconds, so that
            # the main code can check the "alive" flag and respond to SIGINT.
            [r, w, x] = select.select([self.fd], [], [self.fd], 0.1)
            if r:
                return os.read(self.fd, self.bufsize)
            elif x:
                return ''
            else:
                return None

    class MySerial(serial.Serial):
        def nonblocking_read(self, size=1):
            [r, w, x] = select.select([self.fd], [], [self.fd], self._timeout)
            if r:
                try:
                    return os.read(self.fd, size)
                except OSError as e:
                    if e.errno == errno.EAGAIN:
                        return None
                    raise
            elif x:
                raise SerialException("exception (device disconnected?)")
            else:
                return None # timeout

else:
    raise ("Sorry, no terminal implementation for your platform (%s) "
           "available." % sys.platform)

class JimtermColor(object):
    def __init__(self):
        self.setup(1)
    def setup(self, total):
        if total > 1:
            self.codes = [
                "\x1b[1;36m", # cyan
                "\x1b[1;33m", # yellow
                "\x1b[1;35m", # magenta
                "\x1b[1;31m", # red
                "\x1b[1;32m", # green
                "\x1b[1;34m", # blue
                "\x1b[1;37m", # white
                ]
            self.reset = "\x1b[0m"
        else:
            self.codes = [""]
            self.reset = ""
    def code(self, n):
        return self.codes[n % len(self.codes)]

class Jimterm:
    """Normal interactive terminal"""

    def __init__(self,
                 serials,
                 suppress_write_bytes = None,
                 suppress_read_firstnull = True,
                 transmit_all = False,
                 add_cr = False,
                 send_cr = False,
                 raw = True,
                 color = True,
                 bufsize = 65536):

        self.color = JimtermColor()
        if color:
            self.color.setup(len(serials))

        self.serials = serials
        self.suppress_write_bytes = suppress_write_bytes
        self.suppress_read_firstnull = suppress_read_firstnull
        self.last_color = ""
        self.threads = []
        self.transmit_all = transmit_all
        self.add_cr = add_cr
        self.send_cr = send_cr
        self.raw = raw
        self.bufsize = bufsize
        self.quote_re = None
        self.output_lock = threading.Lock()

        self.simulation = WisSimulation()

    def print_header(self, nodes, bauds, output = sys.stdout):
        for (n, (node, baud)) in enumerate(zip(nodes, bauds)):
            output.write(self.color.code(n)
                         + node + ", " + str(baud) + " baud"
                         + self.color.reset + "\n")
        if sys.stdin.isatty():
            output.write("^C to exit\n")
            output.write("----------\n")
        output.flush()

    def start(self):
        self.alive = True

        # Set up console
        self.console = Console(self.bufsize)

        # serial->console, all devices
        for (n, serial) in enumerate(self.serials):
            self.threads.append(threading.Thread(
                target = self.reader,
                args = (serial, self.color.code(n))
                ))

        # console->serial
        self.threads.append(threading.Thread(target = self.writer))

        # start all threads
        for thread in self.threads:
            thread.daemon = True
            thread.start()

    def stop(self):
        self.alive = False

    def join(self):
        for thread in self.threads:
            while thread.is_alive():
                thread.join(0.1)

    def quote_raw(self, data):
        if self.quote_re is None:
            matcher = '[^%s]' % re.escape(string.printable + "\b")
            if sys.version_info < (3,):
                self.quote_re = re.compile(matcher)
                qf = lambda x: ("\\x%02x" % ord(x.group(0)))
            else:
                self.quote_re = re.compile(matcher.encode('ascii'))
                qf = lambda x: ("\\x%02x" % ord(x.group(0))).encode('ascii')
            self.quote_func = qf
        return self.quote_re.sub(self.quote_func, data)

    def reader(self, serial, color):
        """loop and copy serial->console"""

        first = True
        try:
            if (sys.version_info < (3,)):
                null = '\x00'
            else:
                null = b'\x00'
            while self.alive:
                #print(serial.port.split('/')[2].split('.')[1])
                data = serial.nonblocking_read(self.bufsize)
                if data is None:
                    continue
                if not len(data):
                    raise Exception("read returned EOF")

                # don't print a NULL if it's the first character we
                # read.  This hides startup/port-opening glitches with
                # some serial devices.
                if self.suppress_read_firstnull and first and data[0] == null:
                    first = False
                    data = data[1:]
                first = False

                self.output_lock.acquire()

                if color != self.last_color:
                    self.last_color = color
                    os.write(sys.stdout.fileno(), color)

                if self.add_cr:
                    if sys.version_info < (3,):
                        data = data.replace('\n', '\r\n')
                    else:
                        data = data.replace(b'\n', b'\r\n')

                if not self.raw:
                    data = self.quote_raw(data)

                # lines = data
                #
                # # TODO: clean up quick and dirty solution if it works
                #
                # #print(">", data)
                #
                # for line in lines.decode("utf-8").rstrip().split('\n'):
                #
                #     data = line.split(',')
                #
                #     # skip error messaged
                #     if (len(data) > 1):
                #         sensor = int(data[0])
                #         if (sensor in sensors_data.keys()):
                #             # python dict is thread safe https://docs.python.org/3/glossary.html#term-global-interpreter-lock
                #             try:
                #                 sensors_data[sensor]['s1'] = int(data[1])
                #                 sensors_data[sensor]['s2'] = int(data[2])
                #                 sensors_data[sensor]['a'] = int(data[3])
                #             except:
                #                 print("An exception occurred while converting serial data")
                #                 print(">",line,"<")
                #
                #             if (sensor == sensor_to_sync):
                #                 log_sensors()
                #     else:
                #         print("sync")

                #os.write(sys.stdout.fileno(), data)
                files_to_log[serial.port.split('/')[2].split('.')[1]]["file"].write(data)

                # data received
                #print(type(data))
                commands = data.decode("utf-8").rstrip().split("\n")
                for c in commands:
                    command = c.rstrip().split(",")
                    #print(command)
                    assert len(command) == 4, "Command not correct received (serial problem?)"
                    self.simulation.handleMessage(command[0], int(command[1]), int(command[2]), int(command[3]), serial)

                self.output_lock.release()

        except Exception as e:
            self.console.cleanup()
            sys.stdout.write(color)
            sys.stdout.flush()
            traceback.print_exc()
            sys.stdout.write(self.color.reset)
            sys.stdout.flush()
            os._exit(1)

    def writer(self):
        """loop and copy console->serial until ^C"""
        try:
            if (sys.version_info < (3,)):
                ctrlc = '\x03'
            else:
                ctrlc = b'\x03'
            while self.alive:
                try:
                    c = self.console.getkey()
                except KeyboardInterrupt:
                    self.stop()
                    return
                if c is None:
                    # No input, try again.
                    continue
                elif self.console.tty and ctrlc in c:
                    # Try to catch ^C that didn't trigger KeyboardInterrupt
                    self.stop()
                    return
                elif c == '':
                    # Probably EOF on input.  Wait a tiny bit so we can
                    # flush the remaining input, then stop.
                    time.sleep(0.25)
                    self.stop()
                    return
                else:
                    # Remove bytes we don't want to send
                    if self.suppress_write_bytes is not None:
                        c = c.translate(None, self.suppress_write_bytes)

                    if self.send_cr and c == '\n':
                            c = '\r'
                    # Send character
                    if self.transmit_all:
                        for serial in self.serials:
                            serial.write(c)
                    else:
                        self.serials[0].write(c)
        except Exception as e:
            self.console.cleanup()
            sys.stdout.write(self.color.reset)
            sys.stdout.flush()
            traceback.print_exc()
            os._exit(1)

    def run(self):
        # Set all serial port timeouts to 0.1 sec
        saved_timeouts = []
        for serial in self.serials:
            saved_timeouts.append(serial.timeout)
            serial.timeout = 0.1

        # Work around https://sourceforge.net/p/pyserial/bugs/151/
        saved_writeTimeouts = []
        for serial in self.serials:
            saved_writeTimeouts.append(serial.writeTimeout)
            serial.writeTimeout = 1000000

        # Handle SIGINT gracefully
        signal.signal(signal.SIGINT, lambda *args: self.stop())

        # Go
        self.start()
        self.join()

        # Restore serial port timeouts
        for (serial, saved) in zip(self.serials, saved_timeouts):
            serial.timeout = saved
        for (serial, saved) in zip(self.serials, saved_writeTimeouts):
            serial.writeTimeout = saved

        # Cleanup
        sys.stdout.write(self.color.reset + "\n")
        self.console.cleanup()

if __name__ == "__main__":
    import argparse
    import re

    formatter = argparse.ArgumentDefaultsHelpFormatter
    description = ("Simple serial terminal that supports multiple devices.  "
                   "If more than one device is specified, device output is "
                   "shown in varying colors.  All input goes to the "
                   "first device.")
    parser = argparse.ArgumentParser(description = description,
                                     formatter_class = formatter)

    # parser.add_argument("device", metavar="DEVICE", nargs="+",
    #                     help="Serial device.  Specify DEVICE@BAUD for "
    #                     "per-device baudrates.")

    parser.add_argument("--quiet", "-q", action="store_true",
                        help="Don't print header")

    parser.add_argument("--baudrate", "-b", metavar="BAUD", type=int,
                        help="Default baudrate for all devices", default=115200)
    parser.add_argument("--crlf", "-c", action="store_true",
                        help="Add CR before incoming LF")
    parser.add_argument("--lfcr", "-C", action="store_true",
                        help="Send CR instead of LF on output")
    parser.add_argument("--all", "-a", action="store_true",
                        help="Send keystrokes to all devices, not just "
                        "the first one")
    parser.add_argument("--mono", "-m", action="store_true", default=True,
                        help="Don't use colors in output")
    parser.add_argument("--flow", "-f", action="store_true",
                        help="Enable RTS/CTS flow control")
    parser.add_argument("--esp", "-e", action="store_true",
                        help="Force RTS and DTR high, for ESP boards. Note that"
                        " the lines may still glitch low at startup.")
    parser.add_argument("--bufsize", "-z", metavar="SIZE", type=int,
                        help="Buffer size for reads and writes", default=65536)

    group = parser.add_mutually_exclusive_group(required = False)
    group.add_argument("--raw", "-r", action="store_true",
                       default=argparse.SUPPRESS,
                       help="Output characters directly "
                       "(default, if stdout is not a tty)")
    group.add_argument("--no-raw", "-R", action="store_true",
                       default=argparse.SUPPRESS,
                       help="Quote unprintable characters "
                       "(default, if stdout is a tty)")

    args = parser.parse_args()

    piped = not sys.stdout.isatty()
    raw = "raw" in args or (piped and "no_raw" not in args)

    devs = []
    nodes = []
    bauds = []

    for d in files_to_log.keys():
        files_to_log[d]["file"] = open(files_to_log[d]["filename"], 'wb')

    for (n, device) in enumerate(firefly_devices()):
        m = re.search(r"^(.*)@([1-9][0-9]*)$", device)
        if m is not None:
            node = m.group(1)
            baud = m.group(2)
        else:
            node = device
            baud = args.baudrate
        if node in nodes:
            sys.stderr.write("error: %s specified more than once\n" % node)
            raise SystemExit(1)
        try:
            dev = MySerial(None, baud, rtscts = args.flow)
            dev.port = node
            if args.esp:
                # Force DTR and RTS high by setting to false
                dev.dtr = False
                dev.rts = False
            dev.open()
        except serial.serialutil.SerialException:
            sys.stderr.write("error opening %s\n" % node)
            raise SystemExit(1)
        nodes.append(node)
        bauds.append(baud)
        devs.append(dev)

    term = Jimterm(devs,
                   transmit_all = args.all,
                   add_cr = args.crlf,
                   send_cr = args.lfcr,
                   raw = raw,
                   color = (os.name == "posix" and not args.mono),
                   bufsize = args.bufsize)
    if not args.quiet:
        term.print_header(nodes, bauds, sys.stderr)

    term.run()
