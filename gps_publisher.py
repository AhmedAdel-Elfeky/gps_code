#! /usr/bin/env python

import rospy
import time
#import board
#import busio
import adafruit_gps
import serial
from sensor_msgs.msg import NavSatFix


uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart)     # Use UART/pyserial
#gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
#gps.send_command(b'PMTK220,1000')
timestamp = time.monotonic()

rospy.init_node('gps_node',anonymous = True)
pub = rospy.Publisher('/gps_data',NavSatFix , queue_size =10)
rate =rospy.Rate(10)

gps_data = NavSatStatus()
cv = [25,]*9
while True:
        data = gps.read(300)  # read up to 300 bytes
        if data is not None:
                data_string = ''.join([chr(b) for b in data])
                result = data_string.find('GNRMC')
                #print(result)
                if (result >= 0 ) & (result <(300-43)):
                        hour = int(data_string[result+6:result+8])
                        hour = hour +2
                        if hour == 25:
                                hour = 1
                        elif hour == 24:
                                hour = 0
                        minute = data_string[result+8:result+10]
                        sec = data_string[result+10:result+12]
                        ValidFlag = data_string[result+16]
                        print('time is   : ',hour ,minute ,sec)
                        if ValidFlag == 'A':  #data is valid
                                print("Data is valid")
                                Latitude = data_string[result+18:result+20]+'.'+data_string[result+20:result+22]\
				+data_string[result+23:result+28]
                                Longitude = data_string[result+31:result+34]+'.'+data_string[result+34:result+36]\
				+data_string[result+37:result+41]
				if (index_GNGGA >= 0 ) & (index_GNGGA <(300-58)):
                                	altitude = data_string[index_GNGGA+53:index_GNGGA+57]
                                else:
                                	altitude = '0'
                                if (Latitude != '.') & (Longitude != '.') & (altitude != '.'):
                                        Longitude = float(Longitude)
                                        Latitude = float(Latitude)
					index = altitude.find(',')
                                        if index != -1:
                                        	altitude = altitude[:index]
                                        altitude = float(altitude)
					#set the parameters of message
                                        gps_data.latitude=Latitude
                                        gps_data.longitude=Longitude
					gps_data.altitude= altitude
                                        gps_data.header.frame_id = "/gps_frame"
                                        gps_data.status.status = 0
                                        gps_data.status.service = 1
                                        gps_data.header.stamp = timestamp
                                        gps_data.position_covariance = cv
                                        gps_data.position_covariance_type = 1
                                        pub.publish(gps_data)                            			
                                        print('Latitude  :',Latitude)
                                        print('Longitude :',Longitude)
					print('Altitude  :',altitude)
                                else:   
                                	print("Data is not valid")

        if time.monotonic() - timestamp > 5:
                #gps.send_command(b'PMTK605') 
                timestamp = time.monotonic() 
                
        while not rospy.is_shutdown():

			rate.sleep()        

                 



