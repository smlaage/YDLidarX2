""" Module ydlidar_x2
    Driver sofware for the YD LiDAR X2
    SLW - January 2023
"""

import serial
import math
import numpy as np
import time
import tkinter as tk
import warnings
import threading
    

class YDLidarX2:
    
    def __init__(self, port, chunk_size=2000):
        self.__version = 1.03
        self._port = port                # string denoting the serial interface
        self._ser = None
        self._chunk_size = chunk_size    # reasonable range: 1000 ... 10000
        self._min_range = 10			 # minimal measurable distance
        self._max_range = 8000			 # maximal measurable distance
        self._max_data = 20              # maximum number of datapoints per angle
        self._out_of_range = 32768       # indicates invalid data
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
        # predefined list of angle corrections for distances from 0 to 8000
        self._corrections = np.array([0.0] + [math.atan(21.8*((155.3-dist)/(155.3*dist)))*(180/math.pi) for dist in range(1, 8001)])
        # measured distances for angles from 0 to 359
        self._result = np.array([self._out_of_range for _ in range(360)], dtype=np.int32)
        # operating variables for plot functions
        self._org_x, self._org_y = 0, 0
        self._scale_factor = 0.2
        # constants -----
        # array of boundary angles for each sector
        self._sector40_lst = np.array([angle for angle in range(0, 361, 9)], dtype=np.int32)
        self._sector20_lst = np.array([angle for angle in range(0, 361, 18)], dtype=np.int32)
        # array of midpoint angles for each sector
        self._sector40_midpoints = np.arange(4.5, 360.0,  9.0)
        self._sector20_midpoints = np.arange(9.0, 360.0, 18.0)
        # arrays with pre-calculated sinus and cosinus
        self._sin_x = np.array([math.sin(x * math.pi / 180) for x in range(-180, 180)])
        self._cos_x = np.array([math.cos(x * math.pi / 180) for x in range(-180, 180)])
        
        
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
        """ Returns an array of minimum distances for sectors 0 ... 39.
            Resets availability flag.
            Sectors are:
            - sectors[ 0] ->   0 -   8 degree,
            - sectors[ 1] ->   9 -  17 degree,
            - sectors[ 2] ->  18 -  26 degree,
              ...
            - sectors[38] -> 342 - 350 degree,
            - sectors[39] -> 351 - 359 degree 
            Sectors with missing values are reset to the minimum range.
            """
        if not self._is_scanning:
            warnings.warn("get_sectors40: Lidar is not scanning", RuntimeWarning)
       
        self._lock.acquire()
        sectors = np.array([self._result[_ * 9 : _ * 9 + 9].min() for _ in range(40)])
        sectors[sectors > self._max_range] = self._min_range
        self._availability_flag = False
        self._lock.release()
        return sectors
    
    
    def get_sectors20(self):
        """ Returns an array of minimum distances for sectors 0 ... 19.
            Resets availability flag.
            Sectors are:
            - sectors[ 0] ->   0 -  17 degree,
            - sectors[ 1] ->  18 -  35 degree,
            - sectors[ 2] ->  36 -  54 degree,
              ...
            - sectors[38] -> 324 - 341 degree,
            - sectors[39] -> 342 - 359 degree 
            Sectors with missing values are reset to the minimum range.
            """
        if not self._is_scanning:
            warnings.warn("get_sectors20: Lidar is not scanning", RuntimeWarning)
        
        self._lock.acquire()
        sectors = np.array([self._result[_ * 18 : _ * 18 + 18].min() for _ in range(20)])
        sectors[sectors > self._max_range] = self._min_range
        self._availability_flag = False
        self._lock.release()
        return sectors
    
    
    def plot_data(self, cv, dist_measure=None, angle_limit=30):
        """ Plots the current data. Arguments:
            cv -> TKinter canvas
            dist_meaure -> one or more distances (int/float) to be plotted as circle around the origin
            angle_limit -> range of angles to be plotted (int, 0 ... 179) """
        width, height = cv.winfo_reqwidth(), cv.winfo_reqheight()
        self._org_x, self._org_y = width // 2, height * 8 // 10
        # Plot static elements
        self._plot_statics(cv)
        if dist_measure:
            self._plot_dist_measure(cv, dist_measure)
        # plot distance data
        dist_data = self.get_data()
        angle = angle_limit
        while dist_data[angle] == self._out_of_range and angle < 359:
            angle += 1
        if angle >= 359:
            warnings.warn("plot_data: no data available!", RuntimeWarning)
            return
        
        p1_x, p1_y = self._xy_coords(dist_data[angle], angle)
        for a in range(angle + 1, 360 - angle_limit):
            dist = dist_data[a]
            if dist < self._out_of_range:
                p2_x, p2_y = self._xy_coords(dist, a)
                cv.create_line(p1_x, p1_y, p2_x, p2_y)
                p1_x, p1_y = p2_x, p2_y
        
    
    def plot_sectors40(self, cv, dist_measure=None, sec_limit=5):
        """ Plots 40 sectors. Arguments:
            cv -> TKinter canvas
            dist_meaure -> one or more distances (int/float) to be plotted as circle around the origin
            sec_limit -> range of sectors to be plotted (int, 0 ... 19) """
        width, height = cv.winfo_reqwidth(), cv.winfo_reqheight()
        self._org_x, self._org_y = width // 2, height * 8 // 10
        self._plot_statics(cv)
        if dist_measure:
            self._plot_dist_measure(cv, dist_measure)
        # show sectors
        sec_angles = self._sector40_lst  # List of sector angles
        sectors = self.get_sectors40()
        p2_x = None
        for idx in range(sec_limit, 40 - sec_limit):
            dist = sectors[idx]
            if dist < self._out_of_range:
                p1_x, p1_y = self._xy_coords(dist, sec_angles[idx])
                if p2_x:
                    cv.create_line(p2_x, p2_y, p1_x, p1_y, fill='red', width=1, dash=(2,2))
                p2_x, p2_y = self._xy_coords(dist, sec_angles[idx+1])
                cv.create_line(p1_x, p1_y, p2_x, p2_y, fill='red', width=2)
                pt_x, pt_y = self._xy_coords(dist - 60, round(self._sector40_midpoints[idx]))
                cv.create_text(pt_x, pt_y, text=str(idx), fill='red')
            else:
                p2_x = None


    def plot_sectors20(self, cv, dist_measure=None, sec_limit=3):
        """ Plots 20 sectors. Arguments:
            cv -> TKinter canvas
            dist_meaure -> one or more distances (int/float) to be plotted as circle around the origin
            sec_limit -> range of sectors to be plotted (int, 0 ... 9) """
        width, height = cv.winfo_reqwidth(), cv.winfo_reqheight()
        self._org_x, self._org_y = width // 2, height * 8 // 10
        self._plot_statics(cv)
        if dist_measure:
            self._plot_dist_measure(cv, dist_measure)
        # show sectors
        sec_angles = self._sector20_lst  # List of sector angles
        sectors = self.get_sectors20()
        p2_x = None
        for idx in range(sec_limit, 20 - sec_limit):
            dist = sectors[idx]
            if dist < self._out_of_range:
                p1_x, p1_y = self._xy_coords(dist, sec_angles[idx])
                if p2_x:
                    cv.create_line(p2_x, p2_y, p1_x, p1_y, fill='red', width=1, dash=(2,2))
                p2_x, p2_y = self._xy_coords(dist, sec_angles[idx+1])
                cv.create_line(p1_x, p1_y, p2_x, p2_y, fill='red', width=2)
                pt_x, pt_y = self._xy_coords(dist - 60, round(self._sector20_midpoints[idx]))
                cv.create_text(pt_x, pt_y, text=str(idx), fill='red')
            else:
                p2_x = None
                
                
    def plot_vector(self, cv, dist, angle, fill='blue', width=2, dash=(1,1)):
        """ Plots a vector specified by distance and angle.
            Returns the canvas object (in case of any future need). """
        if self._org_x == 0:
            width, height = cv.winfo_reqwidth(), cv.winfo_reqheight()
            self._org_x, self._org_y = width // 2, height * 8 // 10
        p1_x, p1_y = self._xy_coords(dist, angle)
        return cv.create_line(self._org_x, self._org_y, p1_x, p1_y,
                              fill=fill, width=width, dash=dash)
        
    
    def _plot_dist_measure(self, cv, dist_measure):
        """ Plots one or more distances as circels around the origin """
        if self._org_x == 0:
            width, height = cv.winfo_reqwidth(), cv.winfo_reqheight()
            self._org_x, self._org_y = width // 2, height * 8 // 10
        if isinstance(dist_measure, (int, float)):
            dist_measure = [dist_measure]              
        for dist in dist_measure:
            dist *= self._scale_factor
            arc_coord = (self._org_x - dist, self._org_y - dist,
                         self._org_x + dist, self._org_y + dist)
            angle_range = 45
            cv.create_arc(arc_coord, style='arc',
                          start = -angle_range, extent=180 + 2*angle_range, 
                          outline='green')
        

    def _xy_coords(self, dist, angle):
        """ Calculates a coordinate on the canvas """
        angle = round(angle)
        if angle > 359: angle -= 360
        if angle < 0: angle += 360
        dist *= self._scale_factor
        x = round(dist * self._sin_x[angle]) + self._org_x
        y = self._org_y - round(dist * self._cos_x[angle])
        return x, y
    
    
    def _plot_statics(self, cv):
        """ Plots static elements """
        cv.create_line(self._org_x - 75, self._org_y, self._org_x + 75, self._org_y,
                       fill='black', dash=(2,2))
        cv.create_line(self._org_x, self._org_y - 75, self._org_x, self._org_y + 25,
                       fill='black', dash=(2,2))
        
        
    def set_debug(self, debug_level):
        """ Sets the debug level. Range: 0, 1, or 2 """
        self._debug_level = debug_level
    
    def _get_sector40_lst(self):
        """ Returns an array of the border angles for each of the 40 sectors. """
        return self._sector40_lst
    
    def _get_sector20_lst(self):
        """ Returns an array of the border angles for each of the 20 sectors. """
        return self._sector20_lst
    
    def _get_sector40_midpoints(self):
        """ Returns midpoint angles of all 40 sectors """
        return self._sector40_midpoints
    
    def _get_sector20_midpoints(self):
        """ Returns midpoint angles of all 20 sectors """
        return self._sector20_midpoints

    def _available(self):
        """ Indicates whether a new dataset is available """
        return self._availability_flag
    
    def _get_error_cnt(self):
        """ Returns the error count of last data chunk """
        return self._error_cnt
    
    def _get_connected(self):
        """ Returns the status of the connection to the serial interface """
        return self._is_connected

    def _get_scanning(self):
        """ Returns indicator showing whether scanning is active """
        return self._is_scanning
    
    def _get_version(self):
        """ Returns version number of the driver software """
        return self.__version
    
    def _get_out_of_range(self):
        """ Returns out-of-range value, indicating an invalid data item """
        return self._out_of_range
    
    def _get_scale_factor(self):
        return self._scale_factor
    
    def _set_scale_factor(self, f):
        if 0 < f <= 1:
            self._scale_factor = f
        else:
            warnings.warn("set_scale_factor: statement ignored!\nValue must be in range 0 ... 1", RuntimeWarning)
    
    
    # Properties
    is_connected = property(_get_connected)
    is_scanning = property(_get_scanning)
    out_of_range = property(_get_out_of_range)
    available = property(_available)
    error_cnt = property(_get_error_cnt)
    sector40_lst = property(_get_sector40_lst)
    sector20_lst = property(_get_sector40_lst)
    sector40_midpoints = property(_get_sector40_midpoints)
    sector20_midpoints = property(_get_sector20_midpoints)
    scale_factor = property(_get_scale_factor, _set_scale_factor)
    __version__ = property(_get_version)
        
    
#- main program starts here ----------------------------------------------

# --------------------------------------------------------------------------
if __name__ == "__main__":

    import RPi.GPIO as GPIO

    # Global constants
    PIN_LIDAR_PWR = 21      # GPIO pin to power the LiDAR
    
    def run_show():
        if running:
            if lid.available:
                # find sector with farthest distance
                sectors = lid.get_sectors40()
                max_dist, max_sector = 0, 0
                for idx in range(3, len(sectors)-3+1):
                    dist = sectors[idx]
                    if dist < lid.out_of_range and dist > max_dist:
                        max_dist = dist
                        max_sector = idx
                angle = lid.sector40_midpoints[max_sector]
                
                # plot everything:
                # - data for each angle
                # - sectors40
                # - vector to sector with farthest distance 
                cv.delete("all")
                lid.plot_data(cv, (1000, 2000, 3000))
                lid.plot_sectors40(cv)
                lid.plot_vector(cv, max_dist, angle)
                
        # the show must go on
        root.after(350, run_show)
                
        
    def end_show(event):
        global running
        running = False
        time.sleep(0.5)
        root.destroy()
        
    
    #= main program starts here ======================================================

    # Start LiDAR power
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_LIDAR_PWR, GPIO.OUT)
    GPIO.output(PIN_LIDAR_PWR, GPIO.HIGH)
    time.sleep(0.5)

    # Create window
    root = tk.Tk()
    cv = tk.Canvas(root, width=800, height=800)
    cv.pack()
    
    # Prepare for end of the show
    root.bind("q", end_show)
    root.bind('<Escape>', end_show)
    
    # Create lid object on serial port
    lid = YDLidarX2('/dev/serial0')
    lid.scale_factor = 0.15
    lid.connect()
    lid.start_scan()
    print("LiDAR started")
    
    running = True
    run_show()
    root.mainloop()

    # Stop the LIDAR
    lid.stop_scan()
    time.sleep(1)
    lid.disconnect()
    GPIO.output(PIN_LIDAR_PWR, GPIO.LOW)
    print("LiDAR stoped")
    print("Done")
