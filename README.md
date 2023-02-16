# YDLidarX2
Python driver module for the YDLidar X2

This is a Python module for the YDLidar X2. The module was developed on a Raspberry Pi. 
However, there are no specific Raspi dependencies.
The output of the module is a Numpy array comprising distances for each angle, ranging from 0 to 359. 
The distances are measured in mm and stored as integer. 

Dependencies are:
  - module serial:  Provides a serial port for retrieving the data
  - module math
  - module numpy: The drivers is using numpy arrays extensively in order to provide good performance.
  - module time
  - module warnings
  - module threading: The scanning process runs in the background via a separate thread.
  
# Typical use:

Prerequiste: The lidar is connected a serial interface on your system and power on.

-----

# 1) Import the module
- import ydlidar_x2

# 2) Create a lid object, provide a serial port available on your system
- port = '/dev/serial0'
- lid = ydlidar_x2.YDLidarX2(port)

# 3) Connect to the port and start scanning
- lid.connect()
- lid.start_scan()

# 4) Retrieve data 
The scanning process takes approximately 0.3 secs. The duration depends on the chunk size (see table below).
The property 'available' shows when new data has arrived.

-   try:
-    while True:
-       if lid.available:
-         distances = lid.get_data()
-         # process the distances as needed by your application
-       time.sleep(0.1)
-   except KeyboardInterrupt:
-     pass
  
# 5) Shut down the lidar
When you are done, you should stop the scan and close the port.

- lid.stop_scan()
- lid.disconnect()
- print("Done")

-----

# Missing data:
Sometimes, the lidar delivers missing data, internally indicated as a 0 result. 
The module converts any missing data to "out-of-range", 32768. Before using a data point,
check that it is below out-of-range, otherwise ignore.

#  Data retrieval:
- get_data(): delivers an array of 360 values (angle 0 - 359) with distances in mm (or out-of-range)
- get_sectors40(): delivers an array of 40 values for sectors of 9 degree each. 
                   The array contains minimum distances for all measurements in that range.
- get_sectors20(): delivers an array of 20 values for sectors of 18 degree each. 

For convenience, the module provides the angle boundaries for the sectors as properties:
- sector40_lst
- sector20_lst

# Chunk size:
The chunk size specifies the number of data points used for the detection process. 
Greater chunk size means that the scanning process takes longer, however results in more
average data for each angle, thereby improving reliability of the data.
Smaller chunk sizes provide faster scanning processes, however reduce reliability of the results.
Chunk size is an optional argument when invoking the lid object. Default is 2000, which is a useful compromise.

# Error Count:
The data produced by the lidar show occasional errors. If the driver module disocvers an error
in a dataset, the part of the data is ignored. The module counts the number of errors for each chunk.
The error count is available as a property:
- error_cnt
