#!/usr/bin/env python3

'''
 QTrobot gesture utility
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''
import time
import rospy
from qt_gesture_controller.srv import *
from qt_motors_controller.srv import *

def print_help():
    print("QTrobot gesture utility - (C) 2018 LuxAI S.A")
    print("Usage:")
    print("      qt_gesture list                   list available gestures")
    print("      qt_gesture play   <name>          play a gesture given by its <name>")
    print("      qt_gesture record <name> <parts>  record a new gesture. <parts> is a list of 'head', 'right_arm', 'left_arm'")
    print("      qt_gesture home                   move all parts to home postion")
    print("")
    print("Example:")
    print("      $ qt_gesture record mygesture \"rigt_arm, left_arm\"")
    print("      $ qt_gesture play mygesture")
    print("")
# main 
if __name__ == '__main__':
    
    # check the params
    if len(sys.argv) < 2 or sys.argv[1] not in ['list', 'record', 'play', 'home']:
        print_help()
        sys.exit(1)

    if sys.argv[1] == "play" and len(sys.argv) < 3:
        print_help()
        sys.exit(1)

    if sys.argv[1] == "record" and len(sys.argv) < 4:
        print_help()
        sys.exit(1)

    # call the relevant service
    rospy.init_node('qt_gesture_utility')

    if sys.argv[1] == "home":
        try:      
            home = rospy.ServiceProxy('/qt_robot/motors/home', home)
            res = home(["head", "right_arm", "left_arm"])
            if not res.status:
                print("Could move parts to home position")
            time.sleep(2)
        except rospy.ServiceException as e:
            print("Service call failed: %s." % e)

    elif sys.argv[1] == "list":        
        try:      
            gestureList = rospy.ServiceProxy('/qt_robot/gesture/list', gesture_list)
            res = gestureList()
            if res.status:
                print(res.gestures)
        except rospy.ServiceException as e:
            print("Service call failed: %s." % e)
    elif sys.argv[1] == "play":
        try:                  
            gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
            res = gesturePlay(sys.argv[2], 1.0)
            if not res.status:
                print("Could not play gesture '%s'." % sys.argv[2])
        except rospy.ServiceException as e:
            print("Service call failed: %s." % e)
                
    elif sys.argv[1] == "record":
        try:                  
            gestureRecord = rospy.ServiceProxy('/qt_robot/gesture/record', gesture_record)
            gestureSave = rospy.ServiceProxy('/qt_robot/gesture/save', gesture_save)
            setControlMode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
            
            name = sys.argv[2]
            parts = [x.strip() for x in sys.argv[3].split(",")]
            input('Press enter to START recording ...\n')
            res = gestureRecord(parts, True, 0, 0)
            if not res.status:
                print("Could not start recording gesture '%s' using '%s'." % (sys.argv[2], sys.argv[3]))
                sys.exit(1)
            input('Press enter to STOP recording ...\n')
            res = gestureSave(name, "")
            if not res.status:
                print("Could not save gesture '%s'." % sys.argv[2])
            else:
                print("Gesture '%s' recorded." % sys.argv[2])
            res = setControlMode(parts, 1)
            if not res.status:
                print("Could not set control mode of '%s'." % sys.argv[3])
        except rospy.ServiceException as e:
            print("Service call failed: %s." % e)

