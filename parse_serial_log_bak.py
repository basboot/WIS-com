# settings
sps = 64
ts = 1/sps

t = 0

file_in_name = "data.txt"
file_out_name = "20210202_step_gate3_4_s255_no_intake.csv"

sensors_to_log = [201,202,203,204]

sync_found = True # removed sync
sync_count = 0
sensor_count = 0
sensor_data = {}

# open files
file_in = open(file_in_name, 'r')
file_out = open(file_out_name, 'w')

lines = file_in.readlines()

# Strips the newline character
for line in lines:
    data = line.split(',')
    # skip extra newlines at end
    if len(data) < 2:
        continue
    if len(data) > 4:
        print("WARNING > 4")
        continue
    # Synchronisation line found
    if data[0] == "Sync":
        # if already in sync, just skip the message,
        # else count sync messages to see if all sensors are alive
        # TODO: also check sensor numbers
        if not sync_found:
            sync_count += 1

        # all sensors alive
        if sync_count == len(sensors_to_log):
            sync_found = True
    # Dataline found
    else:
        # reset sync count
        # NO reset, syncs not all together every time sync_count = 0
        # only process if not in sync
        if sync_found:
            sensor_data[int(data[0])] = line
            sensor_count += 1

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

file_in.close()
file_out.close()


