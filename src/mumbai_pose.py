import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


x_velo = 0
y_velo = 0
z_velo = 0

def GPS():
    global x_velo, y_velo, z_velo, mumbai_pose
    
    mumbai_pose.pose.position.x = x_velo
    mumbai_pose.pose.position.y = y_velo
    mumbai_pose.pose.position.z = z_velo


def mumbai_direction():
    global x_velo, y_velo, z_velo, mumbai_pose
    mumbai_pose = PoseStamped()
    gps = rospy.Publisher('mumbai_pose', PoseStamped, queue_size=10)
    rospy.init_node('Mumbai_Gps', anonymous=False)
    mumbai_pose.header.frame_id = "odom"
    # x_velo = float(input("Enter x coordinate: "))
    # y_velo = float(input("Enter y coordinate: "))
    x_velo = 4.5
    y_velo = 9.0
    z_velo = 0
    GPS()
    run_rate = rospy.Rate(100)

    
    while not rospy.is_shutdown():
        mumbai_pose.header.stamp = rospy.Time.now()

        gps.publish(mumbai_pose)
        run_rate.sleep()

mumbai_direction()