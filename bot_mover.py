import urx
import math3d as m3d
# import rospy
import time
import numpy as np

from rg2_controller import OnRobotGripperRG2


class BotMover:

    ACC = 0.5
    VEL = 0.2
    # ACC = 0.2
    # VEL = 0.1

    def __init__(self):
        # setup robot
        self.setup()

    def setup(self):
        self.robot = urx.Robot("192.168.199.106")
        self.robot.set_tcp((0, 0, 0.186, 0, 0, 1.5710))
        self.robot.set_payload(2, (0, 0, 0.1))
        time.sleep(2)  #leave some time to robot to process the setup commands

        # setup gripper
        self.gripper = OnRobotGripperRG2(self.robot)

    @property
    def pose_vector(self):
        return self.robot.get_pose().get_pose_vector()

    @property
    def joint_vector(self):
        joint_vector = self.robot.getj(wait=True)
        return joint_vector

    def move_pose(self, pose_vector):
        # get current pose, transform it and move robot to new pose
        self.robot.movel(pose_vector, self.ACC, self.VEL, wait=False)
        while True:
            cur_pv = self.pose_vector
            diff = sum(abs(np.array(cur_pv) - np.array(pose_vector)))
            if diff < 0.001:
                break

    def move_joint(self, joint_vector):
        self.robot.movej(joint_vector, 0.7, 0.5, wait=False)
        while True:
            cur_jv = self.joint_vector
            diff = sum(abs(np.array(cur_jv) - np.array(joint_vector)))
            if diff < 0.001:
                break
    
    def rotate(self, angle):
        pose = self.robot.get_pose()
        pose.orient.rotate_zb(angle)
        pv_to = pose.get_pose_vector()
        self.move_pose(pv_to)

    def grasp(self):
        self.gripper.close_gripper()

    def release(self):
        self.gripper.open_gripper()


if __name__ == '__main__':
    bm = BotMover()
    # print('going up')
    # pv = bm.pose_vector
    # pv[2] += 0.2
    # bm.move_pose(pv)
    # print('going down')
    # pv[2] -= 0.2
    # bm.move_pose(pv)
    import ipdb; ipdb.set_trace()

