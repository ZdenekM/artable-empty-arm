from art_brain import ArtBrainRobotInterface, ArtGripper
import random


class ArtDebugArmInterface(ArtBrainRobotInterface):

    ARM_1 = "arm_1"
    ARM_2 = "arm_2"
    arms = [ARM_1, ARM_2]

    def __init__(self, robot_helper):
        super(ArtDebugArmInterface, self).__init__(robot_helper)

    def select_arm_for_pick(self, obj_id, objects_frame_id, tf_listener):
        #return random.choice(self.arms)
        return self.ARM_1

    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        #return random.choice(self.arms)
        return self.ARM_1

    def select_arm_for_drill(self, obj_to_drill, objects_frame_id, tf_listener):
        #return random.choice(self.arms)
        return self.ARM_1

    def prepare_for_calibration(self):
        pass

    def emergency_stop(self):
        pass

    def restore_robot(self):
        pass

