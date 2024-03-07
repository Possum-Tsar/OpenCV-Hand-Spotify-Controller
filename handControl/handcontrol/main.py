import numpy as np
import cv2
import mediapipe as mp
import math
import subprocess
import pyautogui

#get and initalize mediapipe model
mp_holistic = mp.solutions.holistic
holistic_model = mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

#get and initialize mediapipe drawing utilities
mp_drawing = mp.solutions.drawing_utils

#capture video from camera 0
video = cv2.VideoCapture(0)

#declare some variables
rAvg = None
lAvg = None
lxa = None
lya = None

rtd = 0
rid = 0
rmd = 0
rrd = 0
rpd = 0

ltd = 0
lid = 0
lmd = 0
lrd = 0
lpd = 0

rThumbUp = 0
rIndexUp = 0
rMiddleUp = 0
rRingUp = 0
rPinkeyUp = 0
lThumbUp = 0
lIndexUp = 0
lMiddleUp = 0
lRingUp = 0
lPinkeyUp = 0

rThumbLst = 0
rIndexLst = 0
rMiddleLst = 0
rRingLst = 0
rPinkeyLst = 0
lThumbLst = 0
lIndexLst = 0
lMiddleLst = 0
lRingLst = 0
lPinkeyLst = 0

#program main loop
while(True):
    #capture each video frame
    ret, frame = video.read()

    ##resize frame
    frame = cv2.resize(frame, (800, 600))

    #switch color from BGR to RGB for individual frames
    indivFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    indivFrame.flags.writeable = False
    results = holistic_model.process(indivFrame)
    indivFrame.flags.writeable = True

    #draw left hand landmarks
    mp_drawing.draw_landmarks(indivFrame, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    #draw right hand landmarks
    mp_drawing.draw_landmarks(indivFrame,results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

##########################################################################################################
#hand individual landmark positions
    if results.right_hand_landmarks:
        right_wrist_landmark = results.right_hand_landmarks.landmark[0] # Index 0 is the wrist landmark
        # Get x and y coordinates of the wrist landmark
        rWrist_x = int(right_wrist_landmark.x * frame.shape[1])
        rWrist_y = int(right_wrist_landmark.y * frame.shape[0])
        # Display wrist position in console
        right_thumb_knuckle_landmark = results.right_hand_landmarks.landmark[1]
        rThumb_x = int(right_thumb_knuckle_landmark.x * frame.shape[1])
        rThumb_y = int(right_thumb_knuckle_landmark.y * frame.shape[0])
        right_index_knuckle_landmark = results.right_hand_landmarks.landmark[5]
        rIndex_x = int(right_index_knuckle_landmark.x * frame.shape[1])
        rIndex_y = int(right_index_knuckle_landmark.y * frame.shape[0])
        right_middle_knuckle_landmark = results.right_hand_landmarks.landmark[9]
        rMiddle_x = int(right_middle_knuckle_landmark.x * frame.shape[1])
        rMiddle_y = int(right_middle_knuckle_landmark.y * frame.shape[0])
        right_ring_knuckle_landmark = results.right_hand_landmarks.landmark[13]
        rRing_x = int(right_ring_knuckle_landmark.x * frame.shape[1])
        rRing_y = int(right_ring_knuckle_landmark.y * frame.shape[0])
        right_pinkey_knuckle_landmark = results.right_hand_landmarks.landmark[17]
        rPinkey_x = int(right_pinkey_knuckle_landmark.x * frame.shape[1]) 
        rPinkey_y = int(right_pinkey_knuckle_landmark.y * frame.shape[0]) 
        rxa = (rWrist_x+rThumb_x+rIndex_x+rMiddle_x+rRing_x+rPinkey_x)//6
        rya = (rWrist_y+rThumb_y+rIndex_y+rMiddle_y+rRing_y+rPinkey_y)//6
        #rightHand landmark dists
        rdist1 = math.hypot(rxa  - rThumb_x , rya - rThumb_y)
        rdist2 = math.hypot(rxa - rIndex_x , rya - rIndex_y)
        rdist3 = math.hypot(rxa - rMiddle_x , rya - rMiddle_y)
        rdist4 = math.hypot(rxa - rRing_x , rya - rRing_y)
        rdist5 = math.hypot(rxa - rPinkey_x , rya - rPinkey_y)
        #avg dists right
        rAvg = ((rdist1 + rdist2 + rdist3 + rdist4 + rdist5) / 5) * 2.1
        rCentCoord = (rxa,rya)

        # right finger tips
        #thump
        thumb_tip_landmark = results.right_hand_landmarks.landmark[4]
        rthumbTip_x = int(thumb_tip_landmark.x * frame.shape[1])
        rthumbTip_y = int(thumb_tip_landmark.y * frame.shape[0])
        #pointer
        pointer_tip_landmark = results.right_hand_landmarks.landmark[8]
        rpointerTip_x = int(pointer_tip_landmark.x * frame.shape[1])
        rpointerTip_y = int(pointer_tip_landmark.y * frame.shape[0])
        #middle
        middle_tip_landmark = results.right_hand_landmarks.landmark[12]
        rmiddleTip_x = int(middle_tip_landmark.x * frame.shape[1])
        rmiddleTip_y = int(middle_tip_landmark.y * frame.shape[0])
        #ring
        ring_tip_landmark = results.right_hand_landmarks.landmark[16]
        rringTip_x = int(ring_tip_landmark.x * frame.shape[1])
        rringTip_y = int(ring_tip_landmark.y * frame.shape[0])
        #pinkey
        pinkey_tip_landmark = results.right_hand_landmarks.landmark[20]
        rpinkeyTip_x = int(pinkey_tip_landmark.x * frame.shape[1])
        rpinkeyTip_y = int(pinkey_tip_landmark.y * frame.shape[0])

    if results.left_hand_landmarks:
        left_wrist_landmark = results.left_hand_landmarks.landmark[0]  # Index 0 is the wrist landmark
        # Get x and y coordinates of the wrist landmark
        lWrist_x = int(left_wrist_landmark.x * frame.shape[1])
        lWrist_y = int(left_wrist_landmark.y * frame.shape[0])
        left_thumb_knuckle_landmark = results.left_hand_landmarks.landmark[1]
        lThumb_x = int(left_thumb_knuckle_landmark.x * frame.shape[1])
        lThumb_y = int(left_thumb_knuckle_landmark.y * frame.shape[0])
        left_index_knuckle_landmark = results.left_hand_landmarks.landmark[5]
        lIndex_x = int(left_index_knuckle_landmark.x * frame.shape[1])
        lIndex_y = int(left_index_knuckle_landmark.y * frame.shape[0])
        left_middle_knuckle_landmark = results.left_hand_landmarks.landmark[9]
        lMiddle_x = int(left_middle_knuckle_landmark.x * frame.shape[1])
        lMiddle_y = int(left_middle_knuckle_landmark.y * frame.shape[0])
        left_ring_knuckle_landmark = results.left_hand_landmarks.landmark[13]
        lRing_x = int(left_ring_knuckle_landmark.x * frame.shape[1])
        lRing_y = int(left_ring_knuckle_landmark.y * frame.shape[0])
        left_pinkey_knuckle_landmark = results.left_hand_landmarks.landmark[17]
        lPinkey_x = int(left_pinkey_knuckle_landmark.x * frame.shape[1])
        lPinkey_y = int(left_pinkey_knuckle_landmark.y * frame.shape[0])
        # get centroid of these points, then avg dist for radius from centroid
        lxa = (lWrist_x+lThumb_x+lIndex_x+lMiddle_x+lRing_x+lPinkey_x)//6
        lya = (lWrist_y+lThumb_y+lIndex_y+lMiddle_y+lRing_y+lPinkey_y)//6
        #lefthand landmark dists
        ldist1 = math.hypot(lxa - lThumb_x , lya - lThumb_y)
        ldist2 = math.hypot(lxa - lIndex_x , lya - lIndex_y)
        ldist3 = math.hypot(lxa - lMiddle_x , lya - lMiddle_y)
        ldist4 = math.hypot(lxa - lRing_x , lya - lRing_y)
        ldist5 = math.hypot(lxa - lPinkey_x , lya - lPinkey_y)
        #avg dists
        lAvg = ((ldist1 + ldist2 + ldist3 + ldist4 + ldist5) / 5) * 2.1
        lCentCoord = (lxa, lya)
        # left finger tips
        #thump
        thumb_tip_landmark = results.left_hand_landmarks.landmark[4]
        lthumbTip_x = int(thumb_tip_landmark.x * frame.shape[1])
        lthumbTip_y = int(thumb_tip_landmark.y * frame.shape[0])
        #pointer
        pointer_tip_landmark = results.left_hand_landmarks.landmark[8]
        lpointerTip_x = int(pointer_tip_landmark.x * frame.shape[1])
        lpointerTip_y = int(pointer_tip_landmark.y * frame.shape[0])
        #middle
        middle_tip_landmark = results.left_hand_landmarks.landmark[12]
        lmiddleTip_x = int(middle_tip_landmark.x * frame.shape[1])
        lmiddleTip_y = int(middle_tip_landmark.y * frame.shape[0])
        #ring
        ring_tip_landmark = results.left_hand_landmarks.landmark[16]
        lringTip_x = int(ring_tip_landmark.x * frame.shape[1])
        lringTip_y = int(ring_tip_landmark.y * frame.shape[0])
        #pinkey
        pinkey_tip_landmark = results.left_hand_landmarks.landmark[20]
        lpinkeyTip_x = int(pinkey_tip_landmark.x * frame.shape[1])
        lpinkeytTip_y = int(pinkey_tip_landmark.y * frame.shape[0])
    ############################################################################################################
    #bounding box
    color = (0, 0, 255)
    #if rAvg is not null
    if rAvg is not None:
        cv2.circle(indivFrame, rCentCoord, int(rAvg), color, 2)
    #if lAvg is not null
    if lAvg is not None:
        cv2.circle(indivFrame, lCentCoord, int(lAvg), color, 2)

    #right hand gesture 
    if results.right_hand_landmarks:
        rtd = math.hypot(rthumbTip_x - rxa, rthumbTip_y - rya)
        rid = math.hypot(rpointerTip_x - rxa, rpointerTip_y - rya)
        rmd = math.hypot(rmiddleTip_x - rxa, rmiddleTip_y - rya)
        rrd = math.hypot(rringTip_x - rxa, rringTip_y - rya)
        rpd = math.hypot(rpinkeyTip_x - rxa, rpinkeyTip_y - rya)
        #check finger up
        if rtd > int(rAvg):
            rThumbUp = 1
        if rid > int(rAvg):
            rIndexUp = 1
        if rmd > int(rAvg):
            rMiddleUp = 1
        if rrd > int(rAvg):
            rRingUp = 1
        if rpd > int(rAvg):
            rPinkeyUp = 1
        #check finger down
        if rtd < int(rAvg):
            rThumbUp = 0
        if rid < int(rAvg):
            rIndexUp = 0
        if rmd < int(rAvg):
            rMiddleUp = 0
        if rrd < int(rAvg):
            rRingUp = 0
        if rpd < int(rAvg):
            rPinkeyUp = 0
        #save values for comparison with previous loop, if they match no action, if they dont run gesture
        if rThumbUp == 1 and rThumbLst == 0 and rIndexUp == 1 and rIndexLst == 0: #Open application
            subprocess.run('spotify', check=True)
        if rIndexUp == 1 and rIndexLst == 0 and rPinkeyUp == 1 and rPinkeyLst == 0: #Play or pause song
            pyautogui.hotkey('space')
        if rMiddleUp == 1 and rMiddleLst == 0 and rIndexUp == 1 and rIndexLst == 0: #Turn down volume
            pyautogui.hotkey('ctrl', 'down')
        if rRingUp == 1 and rRingLst == 0: #Play previous song
            pyautogui.hotkey('ctrl', 'right')
        if rMiddleUp == 1 and rMiddleLst == 0:
            pyautogui.hotkey('ctrl', 'left') #Play next song
        if rPinkeyUp == 1 and rPinkeyLst == 0 and rThumbUp == 1 and rThumbLst == 0: #Turn volume up
            pyautogui.hotkey('ctrl', 'up')
        #set last value to current value
        rThumbLst = rThumbUp
        rIndexLst = rIndexUp
        rMiddleLst = rMiddleUp
        rRingLst = rRingUp
        rPinkeyLst = rPinkeyUp
    #left hand gesture
    if results.left_hand_landmarks:
            ltd = math.hypot(lthumbTip_x - lxa, lthumbTip_y - lya)
            lid = math.hypot(lpointerTip_x - lxa, lpointerTip_y - lya)
            lmd = math.hypot(lmiddleTip_x - lxa, lmiddleTip_y - lya)
            lrd = math.hypot(lringTip_x - lxa, lringTip_y - lya)
            lpd = math.hypot(lpinkeyTip_x - lxa, lpinkeytTip_y - lya)
            #check finger up
            if ltd > int(lAvg):
                lThumbUp = 1
            if lid > int(lAvg):
                lIndexUp = 1
            if lmd > int(lAvg):
                lMiddleUp = 1
            if lrd > int(lAvg):
                lRingUp = 1
            if lpd > int(lAvg):
                lPinkeyUp = 1
            #check finger down
            if ltd < int(lAvg):
                lThumbUp = 0
            if lid < int(lAvg):
                lIndexUp = 0
            if lmd < int(lAvg):
                lMiddleUp = 0
            if lrd < int(lAvg):
                lRingUp = 0
            if lpd < int(lAvg):
                lPinkeyUp = 0
            #save values for comparison with previous loop, if they match no action, if they dont run gesture
            if lThumbUp == 1 and lThumbLst == 0:
                pyautogui.hotkey('alt', 'shift', 'B') #Like song
            if lIndexUp == 1 and lIndexLst == 0:
                pyautogui.hotkey('alt', 'shift', '0') #Go to personal library
            if lMiddleUp == 1 and lMiddleLst == 0:
                print("left middle up")
            if lRingUp == 1 and lRingLst == 0:
                print("left ring up")
            if lPinkeyUp == 1 and lPinkeyLst == 0:
                print("left pinkey up")
            #set last value to current value
            lThumbLst = lThumbUp
            lIndexLst = lIndexUp
            lMiddleLst = lMiddleUp
            lRingLst = lRingUp
            lPinkeyLst = lPinkeyUp

    #empty values
    rAvg = None
    lAvg = None
    rCentCoord = None
    lCentCoord = None
    #############################################################################################################
    #switch colors back from RGB to BGR for individual frames
    indivFrame = cv2.cvtColor(indivFrame, cv2.COLOR_RGB2BGR)

    #display each frame
    cv2.imshow('handControl', indivFrame) 

    #quit if q is pressed
    if cv2.waitKey(1) == ord('q'): 
        break

# release capture object
video.release() 

# Destroy all windows 
cv2.destroyAllWindows() 