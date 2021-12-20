import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from geometry_msgs.msg import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion

highway = Twist()
speed = Twist()
speed_forward = speed.linear
dist_meet = 0.20
yaw_thresh = math.pi / 90   #pi/90
yaw = 0
pi = math.pi
pune_pose = []
reached_mumbai = False
check_fuel = False
highway_limit = 0
# left_lane_change_limit = 0
# front_lane_change_limit = 0
car_pose = None
current_pose_x = None
current_pose_y = None
# mumbai_pose_x = 0
# mumbai_pose_y = 0
mumbai_direction = None

class goal_pose():
    # mumbai_gps = Odometry()
    # mum_x = mumbai_gps.pose.pose.position.x = 4.5
    # mum_y = mumbai_gps.pose.pose.position.y = 9.0 
    FIND_PATH = 0
    GOAL_SEEK = 1
    WALL_FOLLOW = 2
# mumbai_pose_x = goal_pose.mum_x
# mumbai_pose_y = goal_pose.mum_y

#Create a new class for all math calculation
car_state = goal_pose.FIND_PATH



def init_main():
    global mumbai
    sleep(0.6)
    rospy.init_node("Bug2", anonymous=True)
    rospy.Subscriber("/mumbai_pose", PoseStamped, init_mumbai)
    rospy.Subscriber("/base_scan", LaserScan, sectors)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, car)
    rospy.spin()

def sectors(laser):
    global front,right_1,right_2,left_1,left_2
    right_1 = min(laser.ranges[0:71])    
    right_2 = min(laser.ranges[70:141])
    front = min(laser.ranges[140:221])
    left_1 = min(laser.ranges[220:291])
    left_2 = min(laser.ranges[290:361])
    # print(front)

    # if front >= 1 and right_1 >= 1 and right_2 >= 1:
    #     highway_limit = 0.6
    #     front_lane_change_limit = 0.6
    #     left_lane_change_limit = 0.9

def range_limits():
    global front_range, right_range_1, right_range_2, left_range_1, left_range_2, right_1, right_2, front, left_1, left_2
    front_range = ()
    right_range_1 = ()
    right_range_2 = ()
    left_range_1 = ()
    left_range_2 = ()
    if front <= 0.6:
        front_range.append(front)
    if right_1 <= 0.6:
        right_range_1.append(right_1)
    if right_2 <= 0.6:
        right_range_2.append(right_2)
    if left_1 <= 0.6:
        left_range_1.append(left_1)
    if left_2 <= 0.6:
        left_range_2.append(left_2)
    # print(front_range)
    # print(left_1)

def car(car_data):
    global car_pose, pune_mumbai_dist, reached_mumbai, mumbai_direction
    car_pose = car_data.pose.pose

    if not check_fuel:
        # print("In fuel_check")
        fuel_check()

    if mumbai_direction is not None:
        car_a = pow(car_pose.position.y - mumbai_direction.y, 2)
        car_b = pow(car_pose.position.x - mumbai_direction.x, 2)
        pune_mumbai_dist = math.sqrt(car_a + car_b)
        # print("Pune-Mumbai Dist",pune_mumbai_dist)
        # print("loop mumbai",reached_mumbai)
        if pune_mumbai_dist <= dist_meet:
            reached_mumbai = True

def init_mumbai(data):
    global mumbai_direction
    mumbai_direction = data.pose.position
    print("MUMBAI",mumbai_direction.x, mumbai_direction.y)
    fuel_check()
    rospy.wait_for_message("/mumbai_pose", PoseStamped)
    
def fuel_check():
    global pose_x, pose_y, car_pose, mumbai_direction, pune_pose, check_fuel
    pose_x = car_pose.position.x
    pose_y = car_pose.position.y
    if car_pose is not None and mumbai_direction is not None:
        check_fuel = True
        pune_pose = [pose_x, pose_y]
        # print("5)Fuel Checked")
        ready_run()

def offset_point():
    global pune_pose, start_point_x,start_point_y, off_line,end_point,pune_pose,car_pose
    start_point_x = pune_pose[0]
    start_point_y = pune_pose[1]
    off_line = car_pose
    end_point = mumbai_direction
    num = math.fabs((end_point.x - start_point_x) * (start_point_y - off_line.position.y) - (start_point_x - off_line.position.x) * (end_point.y - start_point_y))
    # num = math.fabs((end_point.y - start_point_y) * off_line.position.x - (end_point.x - start_point_x) * off_line.position.y + (end_point.x * start_point_y) - (end_point.y*start_point_x))
    deno = math.sqrt(pow(end_point.x - start_point_x, 2) + pow(end_point.y - start_point_y, 2))
    return num/deno          #Distance from point to a line, imaginary line defined by two points


def GOAL_SEEK():
    global current_pose_x, current_pose_y, speed, car_state, front,car_pose, speed_forward, front_range
    alert_collision_front = (front <= 0.8)
    # print("Printing goal_seek")
    if alert_collision_front:
        speed_forward.x = 0
        current_pose_x = car_pose.position.x
        current_pose_y = car_pose.position.y
        car_velocity.publish(speed)
        # print("Changing to Wall Follow now")
        car_state = goal_pose.WALL_FOLLOW
    else:
        speed_forward.x = 4.0
        speed.angular.z = 0
        car_velocity.publish(speed)

def WALL_FOLLOW():
    global car_pose, current_pose_y, current_pose_x, offset_path, car_state, speed_forward
    alert_collision_front = (front <= 0.6)
    safe_left = (left_1 >= 0.9)
    avoid_left = left_2 >= 1.5
    # avoid_left =(left_2 >= 0.4)
    y2_y1 = pow(car_pose.position.y - current_pose_y, 2)         #y2-y1**2
    x2_x1 = pow(car_pose.position.x - current_pose_x, 2)         #x2-x1**2
    offset_path = math.sqrt(y2_y1 + x2_x1)                       
    # print("Printing Wall_follow")
    if offset_point() < 0.25 and offset_path > 0.8:
        # print("line hit")
        # print(offset_path)
        speed_forward.x = 0
        speed.angular.z = 0
        car_velocity.publish(speed)
        # print("Changing to Find Path now")
        car_state = goal_pose.FIND_PATH
        return
    elif avoid_left:
        speed_forward.x = 0
        speed.angular.z = -0.5
        car_velocity.publish(speed)
        # print("Left Avoided") 
    elif safe_left:
        speed_forward.x = 0.3
        speed.angular.z = 0.8
        car_velocity.publish(speed)
        # print("Safe Left")  
    elif alert_collision_front:
        speed_forward.x = 0
        speed.angular.z = -1.0
        car_velocity.publish(speed)
        # print("Alert Collision")
    else:
        speed_forward.x = 4
        speed.angular.z = 0
        car_velocity.publish(speed)
        # print("All Speed Ahead")

def FIND_PATH(current_pose):
    global car_velocity,car_state,yaw_difference, yaw,yaw_thresh, mumbai_pose_yaw,speed, mumbai_direction
    orient_car = (current_pose.orientation.x, current_pose.orientation.y,current_pose.orientation.z, current_pose.orientation.w)
    # print("orient current", current_pose.orientation.z)
    math_euler = euler_from_quaternion(orient_car)
    yaw = math_euler[2]
    
    if math.fabs(yaw) <= 5:
        mumbai_pose_yaw = math.atan2(mumbai_direction.y - current_pose.position.y, mumbai_direction.x - current_pose.position.x)
        
        difference = mumbai_pose_yaw - yaw
        
        if math.fabs(difference)>3.14159:
            difference = difference-(2*math.pi*difference)/(math.fabs(difference))
        yaw_difference = difference

    if math.fabs(yaw_difference) > yaw_thresh:    #yaw_thresh is 0.03490
        speed.angular.z = -0.6
        car_velocity.publish(speed)
    if math.fabs(yaw_difference) <= yaw_thresh:
        speed.angular.z = 0
        # print("Changing to goal_seek now")
        car_velocity.publish(speed)
        car_state = goal_pose.GOAL_SEEK
        
         

def ready_run():
    global car_pose, car_velocity, run_rate
    # velocity()
    car_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    run_rate = rospy.Rate(10)
    # print("6)ready to run") 
    # while not reached_mumbai:
    for i in range(1000):
        if not reached_mumbai:
            # print("8)running in loop")
            if car_state is goal_pose.GOAL_SEEK:
                # print("In Goal Seek Loop")
                GOAL_SEEK()
            elif car_state is goal_pose.WALL_FOLLOW:
                # print("In Wall Follow Loop")
                WALL_FOLLOW()
            elif car_state is goal_pose.FIND_PATH:
                # print("In Find Path Loop")
                FIND_PATH(car_pose)
        run_rate.sleep()
    # print("Reached Mumbai")




if __name__ == "__main__":
    try:
        init_main()
    except rospy.ROSInterruptException:
        pass