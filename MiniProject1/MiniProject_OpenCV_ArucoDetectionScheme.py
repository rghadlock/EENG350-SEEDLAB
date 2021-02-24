# import the necessary packages
import picamera
import picamera.array
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2
import numpy as np
import cv2.aruco as aruco
from cv2 import aruco

#set up lcd screen
lcd_columns = 16
lcd_rows = 2
i2c=busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0,0,100]
time.sleep(1)
bus = smbus.SMBus(1)
address = 4
size = 2

def writeNumber(value):
    bus.write_i2c_block_data(address, 0, value)
    return -1


def readNumber():
    number = bus.read_i2c_block_data(address, 0, size)
    return number


def createMarkers(): #run this program then print the markers that are saved in this program's directory
   aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #set aruco dictionary
   Mk1 = aruco.drawMarker(aruco_dict, 1, 700) #creat marker with ID1 size 700
   Mk2 = aruco.drawMarker(aruco_dict, 2, 700) #creat marker with ID2 size 700
   cv2.imwrite('marker1.jpg', Mk1) #save markers
   cv2.imwrite('marker2.jpg', Mk2)
   cv2.imshow('Marker 1', Mk1) #show markers
   cv2.imshow('Marker 2', Mk2)
   cv2.waitKey(0)
   cv2.destroyAllWindows()
   #the size of the marker must be 700 or the distance will not be accurate

def main():
   camera = PiCamera()
   focalLen  = 1745.95 #pixels (calculated)
   markerHeight = 185 #mm (measured) marker size must be 700. use create marker function
   print("Calibrating Camera...")
   camera.resolution = (1920, 1080) #resolution of computer monitor
   camera.iso = 400 #100 or 200 for daylight and 400 or 800 for low light
   time.sleep(2) #wait for auto gain to settle
   #after waiting for auto settings fix the values
   camera.shutter_speed = camera.exposure_speed 
   camera.exposure_mode = 'off'
   whiteBalance = camera.awb_gains
   camera.awb_mode = 'off'
   camera.awb_gains = whiteBalance
   print("Searching for Marker. Press Ctrl+C to exit")
   try:
      while(1):
         with picamera.array.PiRGBArray(camera) as output:
            camera.capture(output, 'rgb')
            grayImg = cv2.cvtColor(output.array, cv2.COLOR_BGR2GRAY) #get gray image
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #set aruco dictionary
            parameters = aruco.DetectorParameters_create() #create parameters for detector
            #run detection on gray image
            corners, ids, rejectedIMGPoints = aruco.detectMarkers(grayImg, aruco_dict, parameters=parameters)
            if ids is not None: #if marker detected
               print('IDs Detected: ', ids)
               pixHeight = abs(corners[0][0][0][1] - corners[0][0][3][1]) #height of marker in pixels
               distance = focalLen * markerHeight / pixHeight #distance calculation only accurate for size 700 markers
               print("Distance to Marker: ", distance)
               dR = corners[0][0][0][0] - 960 #960 is half 1920 (midle of screen)
               dL = corners[0][0][1][0] - 960
               #print(dR, " ", dL)
               dMid = (dL + dR) /2 #horizontal distance from center of screen to center of marker (x direction)
               angle = dMid * 28/960
               print("Angle: ", angle)
               #determine wheel setpoint
               centerY = (corners[0][0][0][1] + corners[0][0][3][1])/2 #y coordinate of center of marker
               #540 is half of 1080. middle of screen y direction
               if (angle < 0): #left side of image
                  if (centerY < 540): #top half of image
                     wheelPos = 0 #deg = 0 rad
                  else: #bottom half of image
                     wheelPos = 270 #deg = 3pi/2
               else: #right side of image
                  if (centerY < 540): #top half of image
                     wheelPos = 90 #deg = pi/2
                  else: #bottom half of image
                     wheelPos = 180 #deg = pi
               print("Wheel Position (degrees): ", wheelPos)#print degrees
               #show wheel position on lcd
               lcd.clear()
               desireRads = wheelPos * 3.14159 /180
               desireRads = int(desireRads * 1000)
               sendRads = desireRads.to_bytes(2, byteorder = 'big')
               writeNumber(sendRads)
               msg = "Setpoint: %.3f" % (desireRads)
               lcd.message = msg
               ardPos = readNumer()
               curPos = int.from_bytes(ardPos, byteorder = 'big')
               RecievedPOSfromArduino = ((float)curPos * 6.283) / 3200;
               msg = "\nPosition: %.3f" % (RecievedPOSfromArduino)
               lcd.message = msg
            else:
               print("No Markers Found")
            output.truncate(0) #clear image for new capture
            print("\nPress Ctrl+C to exit")
   except KeyboardInterrupt: #stop loop on Ctrl+C
      pass
   
#TODO
   #send setpoint to Arduino
   #Get current position from Arduino
