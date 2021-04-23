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
size = 3

#writes I2C value
def writeNumber(value):
    bus.write_i2c_block_data(address, 0, value)
    return -1

#reads I2C value
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
   state = 0 #state 0 is start
   camera = PiCamera()
   focalLen  = 1745.95 #pixels (calculated)
   markerHeight = 185 #mm (measured) marker size must be 700. use create marker function
   print("Calibrating Camera...")
   camera.resolution = (1920, 1088) #resolution of computer monitor
   time.sleep(2)
   camera.exposure_mode = 'sports'
   camera.awb_mode = 'auto'
   camera.iso = 800
   camera.brightness = 100
   camera.contrast = 100
   camera.sharpness = 100
   camera.shutter_speed = 2000 #works best at 2000 i think
   time.sleep(2) 
   print("Searching for Marker. Press Ctrl+C to exit")
   try:
      while(1):
        with picamera.array.PiRGBArray(camera) as output:
            try:
                if(state == 0 or state == 1 or state ==4):
                    readAbsAngleArd = readNumber()
                    absAngle = int.from_bytes(readAbsAngleArd, byteorder = 'big')
                    absAngle = absAngle >> 8
                    
                    
            except:
                print('i2c error')
            #time.sleep(0.1)
            camera.capture(output, 'rgb')
            grayImg = cv2.cvtColor(output.array, cv2.COLOR_BGR2GRAY) #get gray image
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #set aruco dictionary
            parameters = aruco.DetectorParameters_create() #create parameters for detector
            
#            newSize = (960, 540)
#            newImage = cv2.resize(grayImg, newSize)
#            cv2.imshow('Image', newImage)
#            cv2.waitKey(0)
            
            #run detection on gray image
            corners, ids, rejectedIMGPoints = aruco.detectMarkers(grayImg, aruco_dict, parameters=parameters)
            if ids is not None: #if marker detected
            
               print('IDs Detected: ', ids)
               newPic = True
               pixHeight = abs(corners[0][0][0][1] - corners[0][0][3][1]) #height of marker in pixels
               distance = focalLen * markerHeight / pixHeight #distance calculation only accurate for size 700 markers
               print("Distance to Marker: ", distance)
               dR = corners[0][0][0][0] - 960 #960 is half 1920 (midle of screen)
               dL = corners[0][0][1][0] - 960
               #print(dR, " ", dL)
               dMid = (dL + dR) /2 #horizontal distance from center of screen to center of marker (x direction)
               angle = dMid * 28/960
               print("Angle: ", angle)
               if (state == 0 or state ==1): #if start
                   state = 2 #skip to aim
            else:
               #print("No")
               if (state == 0 ): #if start
                   state = 1 #go to search
                   distance = 0;
                   angle = 0;
            output.truncate(0) #clear image for new capture
            
        #Finite State Machine
        desDistance = int(distance)
        byteDistance = desDistance.to_bytes(2, byteorder = 'big')
        #print(absAngle)
        desAngle = int(absAngle-(angle*100))
        isDesAngleNeg = 0
        if(desAngle < 0):
            desAngle = -1*desAngle
            isDesAngleNeg = 1
      
        byteAngle = desAngle.to_bytes(2, byteorder = 'big')
       
        #Search - 1(rotates until it finds marker)
        if (state == 1):
            #TODO tell arduino to search
           sendBytes = [1, byteDistance[0], byteDistance[1], byteAngle[0], byteAngle[1], isDesAngleNeg]
           try:
                writeNumber(sendBytes)
           except:
                print('i2c error')
        #Aim - 2 (fine tunes angle)
        if (state == 2):
            
            #TODO tell arduino to stop searching
            #TODO send angle to arduino
            sendBytes = [2, byteDistance[0], byteDistance[1], byteAngle[0], byteAngle[1], isDesAngleNeg]
            print(desAngle)
            try:
                writeNumber(sendBytes)
            except:
                print('i2c error')
            time.sleep(2.5)
            state = 3
            #if (angle < 2 and angle > -2):
            #   state = 3
            newPic = False
                   
        #drive - 3(straight)
        elif (state == 3):
            if (newPic == True):
                #TODO send arduino the distance
                sendBytes = [3, byteDistance[0], byteDistance[1], byteAngle[0], byteAngle[1], isDesAngleNeg]
                newPic = False
                try:
                    writeNumber(sendBytes)
                except:
                    print('i2c error')
                #TODO tell arduino to stop aim and start drive
                time.sleep(8)
                cont = readNumber()
                cont1 = cont[0]
                while(cont1 != 1):
                    time.sleep(5)
                    cont = readNumber()
                    cont1 = cont[2]
                print("state4")
                state = 4;
                newPic = False
                    #leave infinite loop stop camera (i think this works)
                
        if(state == 4):
            if (newPic == True):
                print("inState4")
                sendBytes = [2, byteDistance[0], byteDistance[1], byteAngle[0], byteAngle[1], isDesAngleNeg]
                try:
                    writeNumber(sendBytes)
                except:
                    print('i2c error')
                    
   except KeyboardInterrupt: #stop loop on Ctrl+C
      pass

main()