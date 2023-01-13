#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from m2_ps4.msg import Ps4Data
from std_srvs.srv import Empty

old_data = Ps4Data()
acc = 1

def callback(data):
  global old_data
  global acc
  
  # Movement
  cmd_vel = Twist()
  cmd_vel.linear.x = data.hat_ly * acc
  cmd_vel.angular.z = data.hat_rx * acc * 2
  
  # Acceleration
  if(data.dpad_y != old_data.dpad_y and data.dpad_y ** 2 == 1.0):
    acc += int(data.dpad_y)
    if(acc > 6):
      acc = 6
    if(acc < 1):
      acc = 1
    print(f'Acc: {acc}')
      
  pub.publish(cmd_vel)
  
  # Pen Colour
  if(data.triangle != old_data.triangle and data.triangle == True):
    srv_col(0, 255, 0, 2, 0) #green     
  if(data.circle != old_data.circle and data.circle == True):
    srv_col(255, 0, 0, 2, 0) #red     
  if(data.cross != old_data.cross and data.cross == True):
    srv_col(0, 0, 255, 2, 0) #blue 
  if(data.square != old_data.square and data.square == True):      
    srv_col(127, 0, 127, 2, 0) #purple
  
  #Clear 
  if(data.ps != old_data.ps and data.ps == True):
    clear_background()
    
  old_data = data
  
if __name__ == '__main__':
  rospy.init_node('ps4_controller')
  
  # Topics 
  pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
  sub = rospy.Subscriber('input/ps4_data', Ps4Data, callback)
    
  # Services
  rospy.wait_for_service('clear')
  clear_background = rospy.ServiceProxy('clear', Empty)
  
  rospy.wait_for_service('turtle1/set_pen')
  srv_col = rospy.ServiceProxy('turtle1/set_pen', SetPen)
  srv_col(255,255,255,2,0)

  rospy.spin()
