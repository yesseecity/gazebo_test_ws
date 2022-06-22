#!/usr/bin/env python3

# Standard libary imports
import time, os, signal
from threading import Thread

# 3rd party imports
import rospy

# Local modules imports
# Pass

# ROS msg/srv imports
import agvcode.srv as agv_srv
import std_srvs.srv as std_srv
from gazebo_msgs.msg import ContactsState

system_quit = False
contact_state = [0, 0]
def left_bump_response(msg):
  # print("[bumper_left] Length of states: ", len(msg.states))
  if len(msg.states)>0:
    contact_state[0] = 1
  else:
    contact_state[0] = 0

def right_bump_response(msg):
  # print("[bumper_right] Length of states: ", len(msg.states))
  if len(msg.states)>0:
    contact_state[1] = 1
  else:
    contact_state[1] = 0

def show_contact_state():
  while not system_quit:
    print("{:14.5f}  ".format(time.time()), contact_state)
    time.sleep(0.1)

def quit():
  os._exit(0)

def main():
  rospy.init_node('electrode-contact')

  signal.signal(signal.SIGINT ,  lambda sig, frame: Thread(target=quit).start())
  signal.signal(signal.SIGTERM,  lambda sig, frame: Thread(target=quit).start())

  rospy.Subscriber('/charge_station/bumper_left', ContactsState, left_bump_response, queue_size=1)
  rospy.Subscriber('/charge_station/bumper_right', ContactsState, right_bump_response, queue_size=1)
  Thread(target=show_contact_state).start()
  rospy.spin()

if __name__ == '__main__':
    main()