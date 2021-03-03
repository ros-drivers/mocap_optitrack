import rospy
import PyKDL
import tf_conversions.posemath as tfconv

from geometry_msgs.msg import TransformStamped
from aescape_description.srv import SetTransform
from aescape_python.util.tf_helper import TFHelper
from mocap_optitrack.util.constants import OPTITRACK_TRACKER_NAMESPACE
from std_srvs.srv import Trigger, TriggerResponse


class OptitrackCalibrationManager:
    def __init__(self):
        self.tf_helper = TFHelper()
        self.start_services()

    def start_services(self):
        self.calibrate_tracker_service = rospy.Service(
            f"{OPTITRACK_TRACKER_NAMESPACE}/calibrate",
            Trigger,
            self._calibrate_tracker,
        )

    def _calibrate_tracker(self, msg):
        # Get transform from optitrack rigid on link8 to left_arm_base_link
        left_arm_base_to_left_arm_EE = self.tf_helper.get_transform_kdl(
            "left_arm_base_link", "left_arm_link8", rospy.Time.now()
        )

        # Get transform from arm ee to tracker
        left_arm_EE_to_optitrack_link8 = PyKDL.Frame()
        left_arm_EE_to_optitrack_link8.p = PyKDL.Vector(0, 0, 0.03)  # 3cm in Z
        left_arm_EE_to_optitrack_link8.M = PyKDL.Rotation.RotX(3.14)

        # Get transform from optitrack rigid on link8 to optitrack_world
        optitrack_link8_to_optritrack_world = self.tf_helper.get_transform_kdl(
            "optitrack_link8", "optitrack_world", rospy.Time.now()
        )

        # Calculate transform from left_arm_base_link to optitrack_world
        left_arm_base_link_to_optitrack_world = (
            left_arm_base_to_left_arm_EE
            * left_arm_EE_to_optitrack_link8
            * optitrack_link8_to_optritrack_world
        )

        # calculate transform error
        optitrack_link8_to_left_arm_EE = self.tf_helper.get_transform_kdl(
            "optitrack_link8", "left_arm_link8", rospy.Time.now()
        )
        transform_error = (
            left_arm_EE_to_optitrack_link8 * optitrack_link8_to_left_arm_EE
        )
        translation_error = transform_error.p
        rotation_error = transform_error.M.GetRPY()

        rospy.loginfo(
            f"Translation error: {translation_error}, Rotation error: {rotation_error}"
        )

        rospy.loginfo(
            f"left_arm_base_link_to_optitrack_world : {left_arm_base_link_to_optitrack_world}"
        )
        transform_tf2_pose = tfconv.toMsg(left_arm_base_link_to_optitrack_world)
        transform_tf2 = TransformStamped()
        transform_tf2.header.frame_id = "left_arm_base_link"
        transform_tf2.child_frame_id = "optitrack_world"
        transform_tf2.transform.translation.x = transform_tf2_pose.position.x
        transform_tf2.transform.translation.y = transform_tf2_pose.position.y
        transform_tf2.transform.translation.z = transform_tf2_pose.position.z
        transform_tf2.transform.rotation.x = transform_tf2_pose.orientation.x
        transform_tf2.transform.rotation.y = transform_tf2_pose.orientation.y
        transform_tf2.transform.rotation.z = transform_tf2_pose.orientation.z
        transform_tf2.transform.rotation.w = transform_tf2_pose.orientation.w
        rospy.loginfo(f"transform_tf2: {transform_tf2}")

        if not self._set_mutable_transform(transform_tf2):
            return TriggerResponse(
                success=False, message="Could not recalibrate optitrack tracker."
            )

        return TriggerResponse(success=True, message="Optitrack Tracker calibrated.")

    def _set_mutable_transform(self, transform_stamped):
        set_client = rospy.ServiceProxy(
            "/mutable_transform_publisher/setTransform", SetTransform
        )

        try:
            rospy.loginfo("Storing Transform Results: ")
            set_client(transform=transform_stamped)
        except rospy.ServiceException:
            rospy.logerr("Mutable transform publisher not avaliable.")
            return False

        return True
