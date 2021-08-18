# script to flash all fireflies
#
# Usage: python flashff.py subdirectory_with_crystal_bin
#

import sys
import subprocess

assert ((sys.platform  == "darwin") or (sys.platform  == "linux") or (sys.platform == "linux2")), "Platform unsupprted"

if (len(sys.argv) != 2):
    print ("Usage: python flashff.py subdirectory_with_crystal_bin")

dir = str(sys.argv[1])

print("Using image from: %s" % dir)


# executable to find motes
motelist = "./motelist-zolertia"

# alternative version for mac-os
if (sys.platform  == "darwin"):
    motelist = "./motelist-zolertia-macos"

output = (subprocess.Popen([motelist, "-c"], stdout=subprocess.PIPE).communicate()[0]).decode("utf-8")

devices = output.splitlines()

print(devices)
for device in devices:
    print("Flashing device on USB: '%s'" % device.split(",")[1])
    usb = device.split(",")[1]

    flash_output = (subprocess.Popen(["python3", "cc2538-bsl.py", "-e", "-w",  "-v",  "-a 0x00200000",  "-b",  "460800",
                      "-p%s" % usb,  "%s/crystal-test.bin" % dir ], stdout=subprocess.PIPE).communicate()[0]).decode("utf-8")

    #print(flash_output)





