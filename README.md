# YDLidarX2
Python driver for the YDLidar X2

This is a Python module for the YDLidar X2. The module was developed on a Raspberry Pi. 
However there are no specific Raspi dependencies.
The output of the module is a Numpy array comprising distances for each angle (ranging from 0 to 359). 
The distances are measured in mm and stored as integer. 

Dependencies are:
  module serial:  Provides a serial port for retrieving the data
  module math
  module numpy: The drivers is using numpy arrays extensivela in order to provide good performance.
  module time
  module warnings
  module threading: The scanning process runs in the background via a separate thread.
  
Typical use:

-----

# 1) Import the module
import ydlidar_x2

# 2) Create a lid object on a serial port available on your system.
port = '/dev/serial0'
lid = ydlidar_x2.YDLidarX2(port)

# 3) Connect to the port and start scanning
lid.connect()
lid.start_scan()

# 4) Retrieve data. 
# Scanning takes approximately 0.3 secs. The duration depends on the chunk size (see table below).
# The flag 'available' shows when new data is available.

try:
  while True:
    if lid.available:
      distances = lid.get_data()
    # process the distances as needed by your application
    time.sleep()
except KeyboardInterrupt:
  pass
  
# 5) When you are done, shut down the lidar by stopping the scan and closing the port
lid.stop_scan()
lid.disconnect()
print(done)

-----

Missing data:
The lidar produces sometimes missing data. Internally that is indicated as a 0 result. 
The module converts any missing data to "out-of-range", 32768. Before using a data point,
check that it is below out-of-range, otherwise ignore.

Methods to retrieve data:
- get_data(): delivers an array with 360 values (angle 0 - 359) with distances in mm (or out-of-range)
- get_sectors40(): delivers an array of 40 values for sectors of 9 degree each. 
                   The array contains minimum distances for all measurements in that range.
- get_sectors20(): delivers an array of 20 values for sectors of 18 degree each. 

For convenience, the module provides that angle boundaries for the sectors with:
- get_sector40_lst()
- get_sector20_lst()

Chunk size:
The chunk size specifies the number of data points used for the detection process. 
Greater chunk size means that the scanning process takes longer, however results in more
avarage data for each angle, thereby improving reliability of the data.
Smaller chunk sizes provide faster scanning processes, however reduce reliability of the results.
Default chunk_size is 2000, which is a useful compromise.

Error Count:
The data provided by the lidar show occasional errors. If the module disocvers an error
in a dataset, the part of the data is ignored. The module counts the number of errors
for each chunk. The rror count cna be retrieved as a property:
- error_cnt
