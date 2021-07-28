# settings
sps = 128
ts = 1/sps

t = 0

files_to_log = {"sensor1": {"filename" : "tmp_sensorlog1.txt", "file": None, "lines": None},
                "sensor2": {"filename" : "tmp_sensorlog2.txt", "file": None, "lines": None},
                "sensor3": {"filename" : "tmp_sensorlog3.txt", "file": None, "lines": None},
                "sensor4": {"filename" : "tmp_sensorlog4.txt", "file": None, "lines": None}}

# TODO: make a switch for line 54-56

file_out_name = "20210709_filtering_off_on_offtake_on_off_raw_only.csv"

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
            if len(data) > 8: #5
                print("WARNING > TOO MANY VALUES ON A LINE")
                continue

            if data[0] == "Sync":
                sync_found = True
            else:
                # output values needed to do identification (APP_SERIAL_NORMAL / CTRL_COMMAND_MANUAL)
                sensor_data[int(data[1])] = "%s,%s,%s,%s" % (
                    data[1], data[4], data[5], data[6])

                # output values needed to show control (APP_SERIAL_NORMAL / CTRL_COMMAND_AUTO)
                # sensor_data[int(data[0])] = "%s,%s,%s,%s,%s,%s" % (
                # data[0], data[1], data[2], data[3], data[4], data[5])

                # # output values needed to show filtering and do identification (APP_SERIAL_NORMAL / CTRL_COMMAND_MANUAL)
                # sensor_data[int(data[1])] = "%s,%s,%s,%s,%s,%s" % (
                # data[1], data[2], data[3], data[4], data[5], data[6])


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


