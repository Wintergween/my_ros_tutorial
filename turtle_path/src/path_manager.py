#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import *

cur_pos = Pose()

def cb_pose(data): # get the current position from subscribing the turtle position
  global cur_pos
  cur_pos = data

def cb_walk(req):  
  vel = Twist()
  
  if(req.distance < 0):
      return False
  
  if(cur_pos.theta < 0.0):
    theta = cur_pos.theta + 2 * pi
  else:
    theta = cur_pos.theta
  
  #global dest_x  #For use with debug function cb_orientation
  #global dest_y
  
  dest_x = req.distance * cos(theta) + cur_pos.x
  dest_y = req.distance * sin(theta) + cur_pos.y
  
  if(dest_x > 11.0 or dest_x < 0.0):
    return False
  elif(dest_y > 11.0 or dest_y < 0.0):
    return False
  
  #print(f'dest x: {dest_x}')
  #print(f'dest y: {dest_y}')
    
  dest_dist = 1.0

  rate = rospy.Rate(100) # 100Hz control loop

  while (dest_dist > 0.01): # control loop
    dest_dist = sqrt((dest_y - cur_pos.y) ** 2 + (dest_x - cur_pos.x) ** 2)
    
    #print(f'dest dist: {dest_dist}')
    
    if(dest_dist >= 0.2):
      vel.linear.x = 10
    elif(dest_dist > 0.02):
      vel.linear.x = 0.25
    elif(dest_dist > 0.0):
      vel.linear.x = 0.01
    else:
      print("Error going Destination")
    
    pub.publish(vel)    
    rate.sleep()

  return True
  
#def cb_orientation(req): # For debug only, need to disable the original cb_orientation
#  dest_diff = sqrt((dest_y - cur_pos.y) ** 2 + (dest_x - cur_pos.x) ** 2)
#  print(f'ex dest diff: {dest_diff}')
#  return True

#def cb_walk(req): # For debug only, need to disable the original cb_walk
#  dist = fmod(targ_rads - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
#  print(f'ex dist rad: {dist}')
#  return True

def cb_orientation(req):

  rate = rospy.Rate(100) # 100Hz control loop
  dist = 1.0
  vel = Twist()
    
  while (dist > 0.01 or dist < -0.01): # control loop
    dist = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
    
    #global targ_rads   #For use with debug function cb_walk
    #targ_rads = req.orientation
    
    if(dist >= 0.2):
      vel.angular.z = 10
    elif(dist > 0.02):
      vel.angular.z = 0.25
    elif(dist > 0.0):
      vel.angular.z = 0.01
    elif(dist <= -0.2):
      vel.angular.z = -10
    elif(dist < -0.02):
      vel.angular.z = -0.5
    elif(dist < 0.0):
      vel.angular.z = -0.01
    else:
      print("Error Setting Orientation")
    
    #print(f'dist rad: {dist}')
    pub.publish(vel)
    rate.sleep()
  
  return True

if __name__ == '__main__':
  rospy.init_node('path_manager')
  
  #Topics
  pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
  sub = rospy.Subscriber('turtle1/pose', Pose, cb_pose)
  
  #Services  
  rospy.Service('walk_distance', WalkDistance, cb_walk)
  rospy.Service('set_orientation', SetOrientation, cb_orientation)
    
  rospy.spin()
