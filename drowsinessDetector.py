#Importing OpenCV Library for basic image processing functions
import cv2
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
#face_utils for basic operations of conversion
from imutils import face_utils
import time
import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning


# Define GPIO to LCD mapping
# LCD_RS = 7
#LCD_E  = 11
#LCD_D4 = 12
#LCD_D5 = 13
#LCD_D6 = 15
#LCD_D7 = 16
buzzer_pin = 35                # Define PIN for LED
LED_PIN = 29                # Define PIN for LED
button_pin = 13
ena_pin = 12  # ENA pin on L298N (motor enable)
in1_pin = 16  # IN1 pin on L298N (motor direction 1)
in2_pin = 18 # IN2 pin on L298N (motor direction 2)
'''
define pin for lcd
'''
# Timing constants
#E_PULSE = 0.0005
#E_DELAY = 0.0005
#delay = 1



# GPIO.setup(LCD_E, GPIO.OUT)  # E
# GPIO.setup(LCD_RS, GPIO.OUT) # RS
# GPIO.setup(LCD_D4, GPIO.OUT) # DB4
# GPIO.setup(LCD_D5, GPIO.OUT) # DB5
# GPIO.setup(LCD_D6, GPIO.OUT) # DB6
# GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(buzzer_pin,GPIO.OUT)   # Set pin function as output
GPIO.setup(LED_PIN,GPIO.OUT)   # Set pin function as output
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ena_pin, GPIO.OUT)  # Motor enable pin as output
GPIO.setup(in1_pin, GPIO.OUT)  # Motor direction pin 1 as output
GPIO.setup(in2_pin, GPIO.OUT)  # Motor direction pin 2 as output

# Define some device constants
#LCD_WIDTH = 16    # Maximum characters per line
#LCD_CHR = True
#LCD_CMD = False
#LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
#LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Set PWM
pwm = GPIO.PWM(buzzer_pin, 1000)  # Frequency = 1000 Hz
# Set up PWM for motor speed control
pwmMotor = GPIO.PWM(ena_pin, 1000)  # PWM on enable pin with 1kHz frequency
pwmMotor.start(0)  # Start with motor off (0% duty cycle)

'''
Function Name :lcd_init()
Function Description : this function is used to initialized lcd by sending the different commands
'''
'''
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
'''
'''
Function Name :lcd_byte(bits ,mode)
Fuction Name :the main purpose of this function to convert the byte data into bit and send to lcd port
'''
'''
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
 
  # GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  # GPIO.output(LCD_D4, False)
  # GPIO.output(LCD_D5, False)
  # GPIO.output(LCD_D6, False)
  # GPIO.output(LCD_D7, False)
  # if bits&0x10==0x10:
  #   GPIO.output(LCD_D4, True)
  # if bits&0x20==0x20:
  #   GPIO.output(LCD_D5, True)
  # if bits&0x40==0x40:
  #   GPIO.output(LCD_D6, True)
  # if bits&0x80==0x80:
  #   GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
  # Low bits
  # GPIO.output(LCD_D4, False)
  # GPIO.output(LCD_D5, False)
  # GPIO.output(LCD_D6, False)
  # GPIO.output(LCD_D7, False)
  # if bits&0x01==0x01:
  #   GPIO.output(LCD_D4, True)
  # if bits&0x02==0x02:
  #   GPIO.output(LCD_D5, True)
  # if bits&0x04==0x04:
  #   GPIO.output(LCD_D6, True)
  # if bits&0x08==0x08:
  #   GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
'''
'''
Function Name : lcd_toggle_enable()
Function Description:basically this is used to toggle Enable pin
'''
'''
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  # GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  # GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
'''
'''
Function Name :lcd_string(message,line)
Function  Description :print the data on lcd 
'''
'''
def lcd_string(message,line):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
'''

 
# Define delay between readings
delay = 5

#Initializing the camera and taking the instance
cap = cv2.VideoCapture(0)

#Initializing the face detector and landmark detector
hog_face_detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#status marking for current state
sleep = 0
drowsy = 0
active = 0
status="Active"  # Initialize status as Active
color=(0,0,0)

def compute(ptA,ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a,b,c,d,e,f):
    up = compute(b,d) + compute(c,e)
    down = compute(a,f)
    ratio = up/(2.0*down)

    #Checking if it is blinked
    if(ratio>0.25):
        return 2
    elif(ratio>0.21 and ratio<=0.25):
        return 1
    else:
        return 0
'''
lcd_init()
lcd_string("welcome ",LCD_LINE_1)
time.sleep(2)
lcd_string("Driver Sleep",LCD_LINE_1)
lcd_string("Detection System",LCD_LINE_2)
time.sleep(2)
'''
try:
    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = hog_face_detector(gray)

        # Move button check inside the loop, before face detection
        if GPIO.input(button_pin) == GPIO.HIGH and status != "SLEEPING !!!" and status != "Drowsy !":
            print("Pressed")
            # Set motor direction (forward)
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
            # Set motor speed to 100% (adjust PWM duty cycle)
            pwmMotor.ChangeDutyCycle(100)
        else:
            # Stop the motor when button is not pressed or driver is drowsy/sleeping
            pwmMotor.ChangeDutyCycle(0)
            print("Not Pressed or Driver Drowsy/Sleeping")

        time.sleep(0.1)  # Small delay to debounce the button
        
        for face in faces:
            x1 = face.left()
            y1 = face.top()
            x2 = face.right()
            y2 = face.bottom()
            #face_frame = frame.copy()
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            landmarks = predictor(gray, face)
            landmarks = face_utils.shape_to_np(landmarks)

            #The numbers are actually the landmarks which will show eye
            left_blink = blinked(landmarks[36],landmarks[37], 
            landmarks[38], landmarks[41], landmarks[40], landmarks[39])
            right_blink = blinked(landmarks[42],landmarks[43], 
            landmarks[44], landmarks[47], landmarks[46], landmarks[45])

            #Now judge what to do for the eye blinks
            if(left_blink==0 or right_blink==0):
                sleep+=1
                drowsy=0
                active=0
                if(sleep>1):
                    status="SLEEPING !!!"
                    print("SLEEPING !!!")
                    pwm.start(50)
                    GPIO.output(LED_PIN,GPIO.HIGH)
                    pwmMotor.ChangeDutyCycle(0)  # Stop motor
                    color = (0,0,255)

            elif(left_blink==1 or right_blink==1):
                sleep=0
                active=0
                drowsy+=1
                if(drowsy>1):
                    status="Drowsy !"
                    pwm.start(50)
                    GPIO.output(LED_PIN,GPIO.HIGH)
                    pwmMotor.ChangeDutyCycle(0)  # Stop motor
                    color = (0,255,255)

            else:
                drowsy=0
                sleep=0
                active+=1
                if(active>1):
                    status="Active"
                    print("Active")
                    pwm.stop()
                    GPIO.output(LED_PIN,GPIO.LOW)
                    color = (0,255,0)

            cv2.putText(frame, status, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color,3)

            for n in range(0, 68):
                (x,y) = landmarks[n]
                cv2.circle(frame, (x, y), 1, (255, 255, 255), -1)

        cv2.imshow("Microsleep Detection", frame)
        #cv2.imshow("Result of detector", face_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

except KeyboardInterrupt:
    # Clean up GPIO pins on exit
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
