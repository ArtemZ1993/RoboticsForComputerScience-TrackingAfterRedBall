###############################################################
# Robotics for Computer Science HIT 2021-22                   #
# Submitted by: Romi Geora, Itay Itzhak ,Artem Zinenko        #
# Tracking after red ball ,Using camera and Raspberry Pi gpio #
###############################################################

### import libraries ###

import RPi.GPIO as gpio
import time
from threading import Lock, Thread
import matplotlib.pyplot as plt
import numpy as np
import cv2


### function area ###

def forward(sec):  # moving forward
    print("forward")
    init()
    gpio.output(13, False)
    gpio.output(19, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(sec)  # given to the Thread sleep and the wheels continue to moving, sec = How long the Thread sleep
    gpio.cleanup()  # restart pins of Raspberry Pi


def Stop(sec):  # Stop the power supply
    print("Stop")
    init()
    gpio.output(13, False)
    gpio.output(19, False)
    gpio.output(23, False)
    gpio.output(24, False)
    time.sleep(sec)  # given to the Thread sleep and the wheels continue to moving, sec = How long the Thread sleep
    gpio.cleanup()  # restart pins of Raspberry Pi


def left_turn(sec):  # give power to the wheels on the right to turn left
    print("right")
    init()
    gpio.output(13, False)
    gpio.output(19, False)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(sec)  # given to the Thread sleep and the wheels continue to moving, sec = How long the Thread sleep
    gpio.cleanup()  # restart pins of Raspberry Pi


def right_turn(sec):  # give power to the wheels on the left to turn right
    print("left")
    init()
    gpio.output(13, False)
    gpio.output(19, True)
    gpio.output(23, False)
    gpio.output(24, False)
    time.sleep(sec)  # given to the Thread sleep and the wheels continue to moving, sec = How long the Thread sleep
    gpio.cleanup()  # restart pins of Raspberry Pi


def init():  # initialize the pins of the Raspberry Pi to output  (GPIO)
    gpio.setmode(gpio.BCM)
    gpio.setup(13, gpio.OUT)
    gpio.setup(19, gpio.OUT)
    gpio.setup(23, gpio.OUT)
    gpio.setup(24, gpio.OUT)


def find_the_ball():  # function that finds the ball and changes the wheels state

    lower_red = np.array([0, 50, 20])  # set red mask
    upper_red = np.array([10, 255, 255])
    kernel = np.ones((5, 5), np.uint8)

    frame_width = 640  # set the new size of frame
    frame_height = 360

    global state_of_wheels

    cap = cv2.VideoCapture(0)  # Opens a camera for video capturing

    while (True):
        ret, frame = cap.read()  # get the new frame

        if ret == True:  # If I got the new frame

            frame = cv2.resize(frame, (frame_width, frame_height))  # resize frame
            bgr = frame.copy()
            bgr = cv2.GaussianBlur(bgr, (11, 11), 5)  # normalize image using Gaussian
            img_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)  # ‏change the color map to hsv

            mask = cv2.inRange(img_hsv, lower_red, upper_red)  # ‏using red mask to find all red object in frame

            closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            closing_GaussianBlur = cv2.GaussianBlur(closing, (11, 11),
                                                    5)  # normalize image using Gaussian after closing

            circles = cv2.HoughCircles(closing_GaussianBlur, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30,
                                       minRadius=5,
                                       maxRadius=1000)  # find the all Circles in the frame (red mask) using "HoughCircles" algorithm

            found_area = 0
            (center_x, center_y, radius) = 0, 0, 0

            if circles is not None:  # if found any red circle object in the frame

                circles = np.round(circles[0, :]).astype("int")
                index_max_circle = np.argmax(circles[:, -1])  # find the largest circle in the image

                (center_x, center_y, radius) = circles[index_max_circle]  # get center (x,y) and radius
                # cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 4) # added circle to the frame
                found_area = np.pi * (radius ** 2)  # calculate the area

            if found_area > 0:  # if we found the object
                ball_location = [found_area, center_x,
                                 center_y]  # create a list with the following items ,found_area = size , (center_x, center_y) = Center position of the object
            else:
                ball_location = None  # not found object set the list None

            # find ball location in the frame and according to this change the state of the wheels
            if ball_location:
                if ((ball_location[0] > 500) and (ball_location[0] < 100000))  # (571 * 360))):
                    if ball_location[1] < (frame_width / 3):
                        cv2.putText(frame, "Turning left", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),
                                    2)  # added Text to the frame
                        set_state_of_wheels(3)  # Turning left
                    elif ball_location[1] > (frame_width / 3 + (frame_width / 3)):
                        cv2.putText(frame, "Turning Right", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),
                                    2)  # added Text to the frame
                        set_state_of_wheels(2)  # Turning Right
                    else:
                        cv2.putText(frame, "Move Forward", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        set_state_of_wheels(1)  # Move Forward
                elif (ball_location[0] < 500):
                    cv2.putText(frame, "Target isn't large enough, stopping", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 2)  # added Text to the frame
                    set_state_of_wheels(0)  # Target isn't large enough, stopping
                else:
                    cv2.putText(frame, "Target large enough, stopping", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 2)  # added Text to the frame
                    set_state_of_wheels(0)  # Target large enough, stopping
            else:
                cv2.putText(frame, "Target not found, stopping", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),
                            2)  # added Text to the frame
                set_state_of_wheels(0)  # Target not found, stopping

            cv2.imshow('final', frame)  # display frame in a window
            k = cv2.waitKey(1) & 0xff
            if k == 27:
                break
        else:
            break

    set_state_of_wheels(0)
    cap.release()  # release input video
    cv2.destroyAllWindows()  # delete output window
    cv2.waitKey(1)


def set_state_of_wheels(I_state):  # change the state of the wheel
    global state_of_wheels
    lock.acquire()  # synchronization
    state_of_wheels = I_state
    lock.release()


lock = Lock()
state_of_wheels = 0
searcher_thread = Thread(target=find_the_ball)  # "searcher_thread" find the ball "using find_the_ball" method
searcher_thread.start()  # started the thread

while (True):
    if state_of_wheels == 1:  # if state_of_wheels = 1 the car moving forward
        forward(0.005)  # 0.005 time for the wheels to move
    elif state_of_wheels == 0:  # if state_of_wheels = 0 the car Stop
        Stop(0.00025)  # 0.00025 Time for the wheels to move
    elif state_of_wheels == 2:  # if state_of_wheels = 2 the car turn right
        right_turn(0.00025)  # 0.00025 time for the wheels to move
    elif state_of_wheels == 3:  # if state_of_wheels = 3 the car turn left
        left_turn(0.00025)  # 0.00025 time for the wheels to move