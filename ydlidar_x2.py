""" Module ydlidar_x2
Driver sofware for the YD LiDAR X2
SLW - January 2023
"""

import serial
import math
import numpy as np
import time
import warnings
import threading
    

class YDLidarX2:
    
    def __init__(self, port, chunk_size=2000):
        self.__version = 1.03
        self._port = port                # string denoting the serial interface
        self._ser = None
        self._chunk_size = chunk_size    # reasonable range: 1000 ... 10000
        self._min_range = 10
        self._max_range = 8000
        self._max_data = 20              # maximum number of datapoints per angle
        self._out_of_range = 32768       
        self._is_connected = False
        self._is_scanning = False
        self._scan_is_active = False
        self._availability_flag = False
        self._debug_level = 0
        self._error_cnt = 0
        self._lock = threading.Lock()
        self._last_chunk = None
        # 2D array capturing the distances for angles from 0 to 359
        self._distances = np.array([[self._out_of_range for _ in range(self._max_data)] for l in range(360)], dtype=np.uint32)
        # 1D array capturing the number of measurements for angles from 0 to 359
        self._distances_pnt = np.array([0 for _ in range(360)], dtype=np.uint32)
        # corrections is a predefined list of angle corrections for distances from 0 to 8000
        self._corrections = np.array([0.0] + [math.atan(21.8*((155.3-dist)/(155.3*dist)))*(180/math.pi) for dist in range(1, 8001)])
        # result is an array of measured distances for angles from 0 to 359
        self._result = np.array([self._out_of_range for _ in range(360)], dtype=np.int32)
        # sector 40 and sector 20 list of angles
        self._sector40_lst = np.array([angle for angle in range(0, 361, 9)], dtype=np.int32)
        self._sector20_lst = np.array([angle for angle in range(0, 361, 9)], dtype=np.int32)
        
        
    def connect(self):
        """ Connects on serial interface """
        if not self._is_connected:
            try:
                self._ser = serial.Serial(self._port, 115200, timeout = 1)
                self._is_connected = True
            except Exception as e:
                print(e)
                self._is_connected = False
        else:
            warnings.warn("connect: LiDAR already connected", RuntimeWarning)
        return self._is_connected
    
    
    def disconnect(self):
        """ Disconnects the serial interface """
        if self._is_connected:
            self._ser.close()
            self._is_connected = False
        else:
            warnings.warn("disconnect: LiDAR not connected", RuntimeWarning)
            
            
    def start_scan(self):
        """ Starts a thread to run the scan process. """
        if not self._is_connected:
            warnings.warn("start_scan: LiDAR not connected", RuntimeWarning)
            return False
        self._is_scanning = True
        self._scan_thread = threading.Thread(target = self._scan)
        self._scan_thread.start()
        self._availability_flag = False
        return True
    
    
    def stop_scan(self):
        """ Stops the thread running the scan process. """
        if not self._is_scanning:
            warnings.warn("stop_scan: LiDAR is not scanning", RuntimeWarning)
            return False
        else:
            self._is_scanning = False
            while not self._scan_is_active:
                time.sleep(0.1)
            time.sleep(self._chunk_size / 6000)		# wait for the last chunk to finish reading
        return True
    
    
    def _scan(self):
        """ Core routine to retrieve and decode lidar data.
            Availaility flag is set after each successful decoding process. """
        self._scan_is_active = True
        while self._is_scanning:
            # Retrieve data
            data = self._ser.read(self._chunk_size).split(b"\xaa\x55")
            if self._last_chunk is not None:
                data[0] = self._last_chunk + data[0]
            self._last_chunk = data.pop()
            # Clear array for new scan
            distances_pnt = np.array([0 for _ in range(360)], dtype=np.uint32)
            error_cnt = 0
            # Decode data
            for idx, d in enumerate(data):
                # Reasonable length of the data slice?
                l = len(d)
                if l < 10:
                    error_cnt += 1
                    if self._debug_level > 0:
                        print("Idx:", idx, "ignored - len:", len(d))
                    continue
                # Get sample count and start and end angle
                sample_cnt = d[1]
                # Do we have any samples?
                if sample_cnt == 0:
                    error_cnt += 1
                    if self._debug_level > 0:
                        print("Idx:", idx, "ignored - sample_cnt: 0")
                    continue
                # Get start and end angle
                start_angle = ((d[2] + 256 * d[3]) >> 1) / 64
                end_angle = ((d[4] + 256 * d[5]) >> 1) / 64
 
                # Start data block
                if sample_cnt == 1:
                    dist = round((d[8] + 256*d[9]) / 4)
                    if self._debug_level > 1:
                        print("Start package: angle:", start_angle, "   dist:", dist)
                    if dist > self._min_range:
                        if dist > self._max_range: dist = self._max_range
                        angle = round(start_angle + self._corrections[dist])
                        if angle < 0: angle += 360
                        if angle >= 360: angle -= 360
                        self._distances[angle][distances_pnt[angle]] = dist
                        if distances_pnt[angle] < self._max_data - 1:
                            distances_pnt[angle] += 1
                        else:
                            if self._debug_level > 0:
                                print("Idx:", idx, " - pointer overflow")
                            error_cnt += 1

                # Cloud data block
                else:
                    if start_angle == end_angle:
                        if self._debug_level > 0:
                            print("Idx:", idx, "ignored - start angle equals end angle for cloud package")
                        error_cnt += 1
                        continue
                    if l != 8 + 2 * sample_cnt:
                        if self._debug_level > 0:
                            print("Idx:", idx, "ignored - len does not match sample count - len:", l, " - sample_cnt:", sample_cnt)
                        error_cnt += 1
                        continue
                    if self._debug_level > 1:
                        print("Cloud package: angle:", start_angle, "-", end_angle)
                    if end_angle < start_angle:
                        step_angle = (end_angle + 360 - start_angle) / (sample_cnt - 1)
                    else:
                        step_angle = (end_angle - start_angle) / (sample_cnt - 1)
                    pnt = 8
                    while pnt < l:
                        dist = round((d[pnt] + 256*d[pnt+1]) / 4)
                        if dist > self._min_range:
                            if dist > self._max_range: dist = self._max_range
                            angle = round(start_angle + self._corrections[dist])
                            if angle < 0: angle += 360
                            if angle >= 360: angle -= 360
                            self._distances[angle][distances_pnt[angle]] = dist
                            if distances_pnt[angle] < self._max_data - 1:
                                distances_pnt[angle] += 1
                            else:
                                if self._debug_level > 0:
                                    print("Idx:", idx, " - pointer overflow")
                                error_cnt += 1
                        start_angle += step_angle
                        if start_angle >= 360: start_angle -= 360
                        pnt += 2
            # calculate result
            if self._debug_level > 0 and error_cnt > 0:
                print("Error cnt:", error_cnt)
            
            for angle in range(360):
                if distances_pnt[angle] == 0:
                    self._result[angle] = self._out_of_range
                else:
                    self._result[angle] = self._distances[angle][:distances_pnt[angle]].mean()
            self._error_cnt = error_cnt
            self._availability_flag = True
        # end of decoding loop        
        self._scan_is_active = False
        
        
    def get_data(self):
        """ Returns an array of distance data (360 values, one for each degree).
            Resets availability flag"""
        if not self._is_scanning:
            warnings.warn("get_data: Lidar is not scanning", RuntimeWarning)
        self._lock.acquire()
        distances = self._result.copy()
        self._availability_flag = False
        self._lock.release()
        return distances
    
    
    def get_sectors40(self):
        """ Returns an array of minimum distances for sectors.
            Resets availability flag.
            Sectors are:
            - sectors[ 0] ->   0 -   8 degree,
            - sectors[ 1] ->   9 -  17 degree,
            - sectors[ 2] ->  18 -  26 degree,
              ...
            - sectors[38] -> 342 - 350 degree,
            - sectors[39] -> 351 - 359 degree 
            """
        if not self._is_scanning:
            warnings.warn("get_sectors40: Lidar is not scanning", RuntimeWarning)
       
        self._lock.acquire()
        sectors = np.array([self._result[_ * 9 : _ * 9 + 9].min() for _ in range(40)])
        self._availability_flag = False
        self._lock.release()
        return sectors
    
    
    def get_sectors20(self):
        """ Returns an array of minimum distances for sectors.
            Resets availability flag.
            Sectors are:
            - sectors[ 0] ->   0 -  17 degree,
            - sectors[ 1] ->  18 -  35 degree,
            - sectors[ 2] ->  36 -  54 degree,
              ...
            - sectors[38] -> 324 - 341 degree,
            - sectors[39] -> 342 - 359 degree 
            """
        if not self._is_scanning:
            warnings.warn("get_sectors20: Lidar is not scanning", RuntimeWarning)
        
        self._lock.acquire()
        sectors = np.array([self._result[_ * 18 : _ * 18 + 18].min() for _ in range(20)])
        self._availability_flag = False
        self._lock.release()
        return sectors    
    
    
    def set_debug(self, debug_level):
        self._debug_level = debug_level
    
    def _get_sector40_lst(self):
        """ Returns an array comprising the border angles for each sector. """
        return self._sector40_lst
    
    def _get_sector20_lst(self):
        """ Returns an array comprising the border angles for each sector. """
        return self._sector20_lst
    
    def _available(self):
        return self._availability_flag
    
    def _get_error_cnt(self):
        return self._error_cnt
    
    def _get_connected(self):
        return self._is_connected

    def _get_scanning(self):
        return self._is_scanning
    
    def _get_version(self):
        return self.__version
    
    def _get_out_of_range(self):
        return self._out_of_range
    
    
    # Properties
    is_connected = property(_get_connected)
    is_scanning = property(_get_scanning)
    out_of_range = property(_get_out_of_range)
    available = property(_available)
    error_cnt = property(_get_error_cnt)
    sector40_lst = property(_get_sector40_lst)
    sector20_lst = property(_get_sector40_lst)
    __version__ = property(_get_version)
        
    
    
#- main program starts here ----------------------------------------------

# --------------------------------------------------------------------------
if __name__ == "__main__":

    import RPi.GPIO as GPIO
    from tkinter import *

    # Global constants
    PIN_LIDAR_PWR = 21      # GPIO pin to power the LiDAR
    SCALE_FACTOR = 0.2      # Scaling of distances to canvas coordinates
    SCALE_LIMIT = 3000      # Upper range of distances fpr canvas
    ANGLE_LIMIT = 80        # Angle range runs from ANGLE_LIMIT to 360 - ANGLE_LIMIT
    SEC_LIMIT = 9           # Sectors run from SEC_LIMIT to 40 - SEC_LIMIT
    WIDTH, HEIGHT = 900, 700         # Canvas size
    ORG_X, ORG_Y = WIDTH // 2, 600    # Midpoint on canvas
    
    #Operating values
    running: bool = True


    def calc_xy(dist, angle):
        """ Calculates a coordinate on the canvas """
        sin_x = np.array([math.sin(x * math.pi / 180) for x in range(-180, 180)])
        cos_x = np.array([math.cos(x * math.pi / 180) for x in range(-180, 180)])
        if dist > SCALE_LIMIT:
            dist = SCALE_LIMIT
        dist *= SCALE_FACTOR
        x = round(dist * sin_x[angle]) + ORG_X
        y = ORG_Y - round(dist * cos_x[angle])
        return x, y
        

    def show_data():
        """ Reads distances and sectors and shows them on canvas """
        
        if lid.available:
            dist_data = lid.get_data()
            err = lid.error_cnt
            if err > 2:
                print("Error cnt:", err)
            sectors = np.array([dist_data[angle * 9 : angle * 9 + 9].min() for angle in range(40)])
        
            # Plot static elements
            cv.delete("all")
            cv.create_line(ORG_X - 75, ORG_Y, ORG_X + 75, ORG_Y, dash=(2,2))
            cv.create_line(ORG_X, ORG_Y - 75, ORG_X, ORG_Y + 25, dash=(2,2))
            for idx, dist in enumerate(range(500, 2501, 500), 1):
                d = round(dist * SCALE_FACTOR)
                arc_coord = (ORG_X - d, ORG_Y - d, ORG_X + d, ORG_Y + d)
                cv.create_arc(arc_coord, start = -15, extent=210, style='arc',
                              outline='blue2' if idx % 2 == 0 else 'sky blue')
                             
            # show distance data
            p1_x, p1_y = calc_xy(dist_data[ANGLE_LIMIT], ANGLE_LIMIT)
            for angle in range(ANGLE_LIMIT + 1, 360 - ANGLE_LIMIT):
                dist = dist_data[angle]
                if dist < lid.out_of_range:
                    p2_x, p2_y = calc_xy(dist, angle)
                    cv.create_line(p1_x, p1_y, p2_x, p2_y)
                    p1_x, p1_y = p2_x, p2_y
                    
            # show sectors
            sec_angles = lid.sector40_lst  # List of sector angles
            p2_x = None
            for idx in range(SEC_LIMIT, 40-SEC_LIMIT):
                dist = sectors[idx]
                if dist < lid.out_of_range:
                    p1_x, p1_y = calc_xy(dist, sec_angles[idx])
                    if p2_x:
                        cv.create_line(p2_x, p2_y, p1_x, p1_y, fill='red', width=1, dash=(2,2))
                    p2_x, p2_y = calc_xy(dist, sec_angles[idx+1])
                    cv.create_line(p1_x, p1_y, p2_x, p2_y, fill='red', width=2)
                    pt_x, pt_y = calc_xy(dist - 60, (sec_angles[idx] + sec_angles[idx+1])//2)
                    cv.create_text(pt_x, pt_y, text=str(idx), fill='red')
                else:
                    p2_x = None

        # the show must go on
        if running:
            root.after(100, show_data)
        
        
    def end_show(event):
        global running
        running = False
        root.destroy()
        
    
    #= main program starts here ======================================================

    # Start LiDAR
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_LIDAR_PWR, GPIO.OUT)
    GPIO.output(PIN_LIDAR_PWR, GPIO.HIGH)
    time.sleep(0.5)

    # Create window
    root = Tk()
    cv = Canvas(root, width = WIDTH, height=HEIGHT)
    cv.pack()
    
    # Prepare for end of the show
    root.bind("q", end_show)
    root.bind('<Escape>', end_show)
    
    # Create lid object on serial port
    lid = YDLidarX2('/dev/serial0')
    lid.set_debug(0)
    lid.connect()
    lid.start_scan()
    print("LiDAR started")
    
    # Run the show
    show_data()
    root.mainloop()

    # Stop the LIDAR
    lid.stop_scan()
    time.sleep(1)
    lid.disconnect()
    GPIO.output(PIN_LIDAR_PWR, GPIO.LOW)
    print("LiDAR stoped")
    print("Done")

