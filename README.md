# WIS-lab

Python scripts for reading and sending data to de WIS lab setup.

## flashff.py

Flashes a binary image to all usb-connected Fireflies.

```python3 flashff.py {directory}```

Put the desired image named ```crystal-test.bin``` in ```{directory}```


## log_terminal.py

Reads all serial sensor data from the Fireflies and stores them in temporary files (tmp_sensorlogX.txt) (set usb ports to in- or exclude on line 32 and 42).

## parse_serial_log.py

Combines the temprary logs into one csv-file with all sensors combines in one line (set csv filename on line 14).

## wis_simulation.py

A hardware-in-the-loop created specific for the WCB-E+ branch to test ETC+.

Note: this simulation can only be connected to the dev-etcp branch of WCB.