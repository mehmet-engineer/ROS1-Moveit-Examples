import rospy
from std_msgs.msg import String

stop_traj_publisher = rospy.Publisher("/trajectory_execution_event", String, queue_size=10)

def stop_execution(self):
    str_msg = String()
    str_msg.data = "stop"
    self.stop_traj_publisher.publish(str_msg)
    rospy.loginfo("Trajectory execution stopped.")
