import sys
import PPM
import RPi.GPIO as GPIO
import time
import firebase_admin
import cv2
import re
import pynmea2

#from folder.file import PPM
import gps
import math
from firebase_admin import credentials
from firebase_admin import firestore
from i2clibraries import i2c_hmc5883l
import pigpio


# Libraries for Arduino pin for drone signalssudo 
throttle, yaw, pitch, roll =  1500, 1500, 1500, 1500 # Require PWM outputs only
ARM, FAILSAFE, ANGLE_MODE, ALTHOLD = 1000, 1000, 1000, 1000 # any GPIO pins would work
avgLat = 0
avgLon = 0


### Setup GPIO Pins on Raspberry Pi ###
GPIO.setmode(GPIO.BCM)
BUZZER = 5
GPIO.setup(BUZZER, GPIO.OUT, initial= GPIO.LOW)
success = 7

def buzz(beeps):
    for i in range(beeps):
        GPIO.output(BUZZER, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(0.1)
    time.sleep(2)

def buzzFailure():
    GPIO.output(BUZZER, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(BUZZER, GPIO.LOW)
    time.sleep(1)

pi = pigpio.pi()
if not pi.connected:
    print("error")
    exit(0)

pi.wave_tx_stop() # Start with a clean slate.
ppm = PPM.X(pi, 6, frame_ms=27) # GPIO 6 for PPM


class flightBehavior:
    GPS_COORDINATES = 1
    HOVER = 2
    NAVIGATE_TO_DESTINATION = 3
    GIVE_SAMPLE = 4
    RETURN_TO_HOME = 5

# important function to send signals to Arduino
def signalParameters(throttle, yaw, pitch, roll, ARM, FAILSAFE, ANGLE_MODE, ALTHOLD):
    print("throttle = ", throttle)
    print("yaw = ", yaw)
    print("pitch = ", pitch)
    print("roll = ", roll)
    print("ARM = ", ARM)
    print("FAILSAFE = ", FAILSAFE)
    print("ANGLE_MODE = ", ANGLE_MODE)
    print("ALTHOLD = ", ALTHOLD)
    print("\n")
    
    ppm.update_channels([throttle, yaw, pitch, roll, ARM, FAILSAFE, ANGLE_MODE, ALTHOLD])
    
# Send these signals via GPIO or implement SPI whatever suits better
# example on how to use SPI -- pretty simple

# spi = spidev.SpiDev()
# spi.open(0, 0)
# spi.max_speed_hz = 1000000 # 1MHz clock speed
# spi.nos_cs = True
# writebytes([0x02, 0x01, 0x03])

currentState = flightBehavior.GPS_COORDINATES
verify = False
while True:
    if(currentState == flightBehavior.GPS_COORDINATES):
        buzz(currentState)
        cred = credentials.Certificate("serviceAccountKey.json")
        firebase_admin.initialize_app(cred)

        db = firestore.client()
        # create drone user
        # db.collection('Users').add({ 'email':'drone@drone.com', 'user_id':'drone', 'username': 'drone'})

        #read data from randomly generated id document
        result = db.collection("User Locations").document("2YxFOdaYK2PjZXsq1gWJiyVRhO73").get()

        geo = result.get('geo_point')
        longitudeDest = geo.longitude
        latitudeDest = geo.latitude
        
        if longitudeDest > -100 and longitudeDest < 100 and latitudeDest > -100 and latitudeDest < 100:
            verify = True
        if (verify):
            currentState = flightBehavior.HOVER
            buzz(success)
            print('destination longitude: ' ,longitudeDest)
            print('destination latitude: ' ,latitudeDest)

            time.sleep(10)
    if(currentState == flightBehavior.HOVER):
        buzz(currentState)
        print("Starting the drone by sending minimum throttle value\n")
        print("Should loop 2 times")
        for i in range(2):
            throttle = 1000
            yaw = 1500 # get accurate values from arduino
            pitch = 1500 # get accurate values from arduino
            roll = 1500 # get accurate values from arduino
            ARM = 2000 # AUX1
            FAILSAFE = 1000 # AUX2
            ANGLE_MODE = 2000 # AUX3
            ALTHOLD = 1000 # AUX4
            signalParameters(throttle, yaw, pitch, roll, ARM, FAILSAFE, ANGLE_MODE, ALTHOLD)
        print("Drone started")
#         The purpose of this loop is to increase the throttle to bring the drone to a height
        print("Now need the drone to hover by increasing the throttle")
        for i in range(1000, 1650, 50):
            throttle = i
            print("Drone throttle", throttle)
            
            signalParameters(throttle, yaw, pitch, roll, ARM, FAILSAFE, ANGLE_MODE, ALTHOLD)
            time.sleep(1)
#             Send these signals via GPIO or implement SPI whatever suits better
        
        print("Drone hold is now in effect")
        
#         Drone hold is now in effect
        ALTHOLD = 2000
        signalParameters(1550, yaw, pitch, roll, ARM, FAILSAFE, ANGLE_MODE, ALTHOLD) 
        buzz(success)
        time.sleep(1)
        signalParameters(1550, yaw, pitch, roll, ARM, 2000, ANGLE_MODE, ALTHOLD);
        currentState = flightBehavior.NAVIGATE_TO_DESTINATION
    
    if (currentState == flightBehavior.NAVIGATE_TO_DESTINATION):
        buzz(currentState)
        print("Navigating to destination\n")

        print("1")
        session = gps.gps("localhost", "2947")
        session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        
        # HARDCODE THIS PROBLEM
        
        # HARDCODE THIS PROBLEM
        print("2")
        while True:
            try:
                print("3")
                report = session.next()
                print("4")
        # Wait for a 'TPV' report and display the current time
        # To see all report data, uncomment the line below
        #print(report)
                latDrone = getattr(report, 'lat', "Unknown")
                lonDrone = getattr(report, 'lon', "Unknown")
        
                latSum = 0
                lonSum = 0
                counter = 0
                for i in range (10):
                    report = session.next()
                    latDrone = getattr(report, 'lat', "Unknown")
                    lonDrone = getattr(report, 'lon', "Unknown")
            
                    if (not lonDrone == 'Unknown') or (not latDrone == 'Unknown'):
                        counter +=1
                        latSum += latDrone
                        lonSum += lonDrone
                
                    if counter != 0:
                        avgLat = latSum/counter
                        avgLon = lonSum/counter
            
                    
                    print("latitude is:" , avgLat)
                    print("longitude is:" , avgLon)
        
                    hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1) #choosing which i2c port to use, RPi2 model B uses port 1
                   
                    hmc5883l.setContinuousMode()
                   
                    hmc5883l.setDeclination(0,6) #type in the magnetic declination of your location in the bracket (degrees, minute)
                   
                   
                    print(hmc5883l)  
                    bearing  = 0.0# heading direction
                    fdL = 0.0# difference in longitude
                    thetaA = 0.0# initial latitude
                    thetaB = 0.0# final latitude
                    La = 0.0# initial longitude
                    Lb= 0.0 # final longitude
                    X = 0.0 # X coordinates
                    Y = 0.0# Y coordinates
                    
                    thetaA = math.radians(avgLat)
                    thetaB = math.radians(latitudeDest)
                    La = avgLon
                    Lb = longitudeDest
                    dL = math.radians(Lb - La)
                    
                    X = math.cos(thetaB)*math.sin(dL)
                    Y = math.cos(thetaA)*math.sin(thetaB) - math.sin(thetaA)*math.cos(thetaB)*math.cos(dL)
                    bearing = math.atan2(X, Y)
                    
                    print(X)
                    print(Y)
                    print('heading direction: ', math.degrees(bearing))
                    
        
       
            except KeyError:
                buzzFailure()
                pass
            except KeyboardInterrupt:
                currentState = flightBehavior.GIVE_SAMPLE
                break
            except StopIteration:
    #             buzzFailure()
                session = None
                print("GPSD has terminated")

        
        
        buzz(success)
        currentState = flightBehavior.GIVE_SAMPLE
        
    if (currentState == flightBehavior.GIVE_SAMPLE):
        buzz(currentState)
        print("Verifying QR code and giving and retreiving samples\n")
        LED_PIN = 12
        GPIO.setup(LED_PIN, GPIO.OUT, initial= GPIO.LOW)

        GPIO.output(LED_PIN,GPIO.HIGH) #LED will be switched on


        cap = cv2.VideoCapture(0)
        detector = cv2.QRCodeDetector()

        print("Reading QR code using Raspberry Pi camera")

        data = ""

        while data == "":

            _, img = cap.read()
        #     print(img)
            data, _, _ = detector.detectAndDecode(img)

        print("Data found: " + data)
        GPIO.output(LED_PIN,GPIO.LOW) #LED will be switched on
        buzz(success)
        cap.release()
        currentState = flightBehavior.RETURN_TO_HOME
        
    if (currentState == flightBehavior.RETURN_TO_HOME):
        buzz(currentState)
        print("Returning Home now\n")
#        
        buzz(success)
    break
    
print("\n\nMission Completed")
buzz(20)