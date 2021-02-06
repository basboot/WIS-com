# settings
sps = 128
ts = 1/sps

t = 0

files_to_log = {"SLAB_USBtoUART3": {"filename" : "SLAB_USBtoUART3.txt", "file": None, "lines": None},
                "SLAB_USBtoUART2": {"filename" : "SLAB_USBtoUART2.txt", "file": None, "lines": None},
                "SLAB_USBtoUART": {"filename" : "SLAB_USBtoUART.txt", "file": None, "lines": None},
                "SLAB_USBtoUART6": {"filename" : "SLAB_USBtoUART6.txt", "file": None, "lines": None}}


file_out_name = "20210202_step_gate3_4_s50_no_intake3.csv"

sensors_to_log = [201,202,203,204]

sync_found = True # removed sync
sync_count = 0
sensor_count = 0
sensor_data = {}

# open files
max_lines = 1000000000

for d in files_to_log.keys():
    files_to_log[d]["file"] = open(files_to_log[d]["filename"], 'r')

    files_to_log[d]["lines"] = files_to_log[d]["file"].readlines()

    max_lines = min(max_lines, len(files_to_log[d]["lines"]))

file_out = open(file_out_name, 'w')


line = 0
for line in range(max_lines):
    sync_found = False

    for d in files_to_log.keys():
        data = files_to_log[d]["lines"][line].split(',')

        # skip extra newlines at end
        if len(data) < 4:
            continue
        else:
            if len(data) > 5:
                print("WARNING > 5")
                continue

            if data[0] == "Sync":
                sync_found = True
            else:
                sensor_data[int(data[0])] = "%s,%s,%s,%s" % (data[0], data[1], data[2], data[3])#files_to_log[d]["lines"][line]
                sensor_count += 1

    if sync_found:
        continue

    # all sensors received
    if sensor_count == len(sensors_to_log):
        # write timestamp
        file_out.write("%s," % int(t * 1000))
        t += ts

        # write sensor data
        for i in range(len(sensors_to_log)):
            file_out.write("%s" % sensor_data[sensors_to_log[i]].rstrip())
            if i < len(sensors_to_log) - 1:
                file_out.write(",")
            else:
                file_out.write("\n")

        sensor_count = 0



for d in files_to_log.keys():
    files_to_log[d]["file"].close()

file_out.close()


