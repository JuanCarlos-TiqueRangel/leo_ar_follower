#! /usr/bin/env python
import rospy

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from tf.transformations import quaternion_from_euler

from std_srvs.srv import Trigger, TriggerResponse

class dynamic_ar(object):
    def __init__(self):

        self.leo_position_x = 0.0
        self.leo_position_y = 0.0
        self.leo_position_z = 0.0
        self.leo_yaw = 0.0
        self.distance_to_leo = 2.0

        #Set up publisher and subscribers
        self.marker_pose_pub = rospy.Publisher('/updated_ar_marker_pose', AlvarMarkers, queue_size=1)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.marker_pose_sub = rospy.Subscriber('/updated_ar_marker_pose', AlvarMarkers, self.ar_marker_pose_callback)

        self.goal_achieved_service = rospy.Service('/goal_trigger_ar', Trigger, self.trigger_function)

        self.update_ar_mark = AlvarMarker()
        self.update_ar_mark_msg = AlvarMarkers()

        self.move_ar_tag = False
        self.count = 0

        self.angular_velocity = 0.1

        self.positions = [
                (2.0, 0.0),
                (2.0, 0.0),
                (3.0, 0.0),
                (4.0, 0.0),
                (3.0, 0.0),
                (2.0, 0.0),
                (2.0, 1.0),
                (2.0, 2.0),
                (2.0, 1.0),
                (2.0, 0.0),
                (2.0, -1.0),
                (2.0, -2.0),
                (2.0, -3.0),
                (2.0, -2.0),
                (2.0, -1.0),
                (2.0, 0.0),
                (4.0, 0.0),
                (3.0, 0.0),
                (2.0, 0.0),
                (1.0, 0.0),
                (0.0, 0.0)
            ]

    def trigger_function(self, request):
        self.move_ar_tag = True
        self.count += 1
        if self.count > len(self.positions):
            message = 'No more targets'
        else:
            message = self.count
        return TriggerResponse(
            success=True,
            message='Target: ' + str(message) 
        )

    def ar_marker_pose_callback(self, msg):
        self.update_simulation(msg)

    def update_simulation(self, pose):
        model_state = ModelState()
        model_state.model_name = 'Marker0'  # Replace with your model's name
        model_state.pose = pose.markers[0].pose.pose

        self.set_model_state(model_state)

    def main(self):
            rate = rospy.Rate(50)

            roll = 0.0  # Replace with roll angle in radians
            pitch = 0.0  # Replace with pitch angle in radians
            yaw = 0.0  # Replace with yaw angle in radians
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            self.update_ar_mark.pose.pose.position.x = 1.0
            self.update_ar_mark.pose.pose.position.y = 0.0
            self.update_ar_mark.pose.pose.position.z = 0.0
            self.update_ar_mark.pose.pose.orientation.x = quaternion[0]
            self.update_ar_mark.pose.pose.orientation.y = quaternion[1]
            self.update_ar_mark.pose.pose.orientation.z = quaternion[2]
            self.update_ar_mark.pose.pose.orientation.w = quaternion[3]
            self.update_ar_mark_msg.markers = [self.update_ar_mark]

            while not rospy.is_shutdown():

                roll = 0.0  # Replace with roll angle in radians
                pitch = 0.0  # Replace with pitch angle in radians
                yaw = 0.0  # Replace with yaw angle in radians
                quaternion = quaternion_from_euler(roll, pitch, yaw)

                if self.move_ar_tag == True and self.count < len(self.positions):
                        x, y = self.positions[self.count]
                        self.update_ar_mark.pose.pose.position.x = x
                        self.update_ar_mark.pose.pose.position.y = y
                        self.update_ar_mark.pose.pose.position.z = 0.0
                        self.update_ar_mark.pose.pose.orientation.x = quaternion[0]
                        self.update_ar_mark.pose.pose.orientation.y = quaternion[1]
                        self.update_ar_mark.pose.pose.orientation.z = quaternion[2]
                        self.update_ar_mark.pose.pose.orientation.w = quaternion[3]
                        self.update_ar_mark_msg.markers = [self.update_ar_mark]
                        print("AR_pose: ", self.count)
                        self.move_ar_tag = False

                self.marker_pose_pub.publish(self.update_ar_mark_msg)
                rate.sleep()  # Control the rate of updates
                    

if __name__=='__main__':
	try:
		rospy.init_node("dynamic_ar_tag")
		print("Nodo dynamic ar Creado")
		cv = dynamic_ar()
		cv.main()

	except rospy.ROSInterruptException:
		pass
