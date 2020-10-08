import subprocess
import sys

assert ((sys.platform  == "darwin") or (sys.platform  == "linux")), "Platform unsupprted"

# executable to find motes
motelist = "./motelist-zolertia"

# alternative version for mac-os
if (sys.platform  == "darwin"):
    motelist = "./motelist-zolertia-macos"

output = (subprocess.Popen(["./motelist-zolertia-macos", "-c"], stdout=subprocess.PIPE).communicate()[0]).decode("utf-8")

devices = output.splitlines()

for device in devices:
    print("Flashing device on USB: %s" % device.split(",")[1])


