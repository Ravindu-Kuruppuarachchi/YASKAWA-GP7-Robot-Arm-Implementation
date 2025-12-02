#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import time

class GP7Controller(Node):
    def __init__(self):
        super().__init__('gp7_controller')

        # --- Robot Setup ---
        L1_d=0.330; L1_a=0.040; L2_a=0.445; L3_u=0.440; L4_d=0.440; L6_d=0.080
        Gripper_Len = 0.14 

        L1 = rtb.RevoluteDH(d=L1_d, a=L1_a, alpha=-np.pi/2)
        L2 = rtb.RevoluteDH(d=0,    a=L2_a, alpha=0, offset=-np.pi/2)
        L3 = rtb.RevoluteDH(d=0,    a=0.040, alpha=-np.pi/2)
        L4 = rtb.RevoluteDH(d=L3_u, a=0,     alpha=np.pi/2)
        L5 = rtb.RevoluteDH(d=0,    a=0,     alpha=-np.pi/2)
        L6 = rtb.RevoluteDH(d=L6_d, a=0,     alpha=0)

        self.gp7 = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='GP7')
        self.gp7.tool = SE3(0, 0, Gripper_Len)

        # --- Arm Publishers ---
        self.pubs = [
            self.create_publisher(Float64, f'/model/gp7_robot/joint/joint_{name}/cmd_pos', 10)
            for name in ['s', 'l', 'u', 'r', 'b', 't']
        ]
        
        # --- Gripper Publishers (Fingers) ---
        self.finger_l = self.create_publisher(Float64, '/model/gp7_robot/joint/finger_joint/cmd_pos', 10)
        self.finger_r = self.create_publisher(Float64, '/model/gp7_robot/joint/right_outer_knuckle_joint/cmd_pos', 10)

        self.timer = self.create_timer(1.0, self.execute_motion)
        self.motion_started = False

    def execute_motion(self):
        if self.motion_started: return
        self.motion_started = True
        self.calculate_and_move()

    def set_gripper(self, closed=False):
        """Controls the gripper fingers"""
        pos = 0.9 if closed else 0.0 # 0.8 rad is closed, 0.0 is open
        msg = Float64()
        msg.data = pos
        
        # Publish multiple times to ensure Gazebo catches it
        action = "CLOSING" if closed else "OPENING"
        self.get_logger().info(f"--> GRIPPER: {action}")
        
        for _ in range(5):
            self.finger_l.publish(msg)
            self.finger_r.publish(msg)
            time.sleep(0.02)

    def move_segment(self, trajectory):
        for q in trajectory:
            for i in range(6):
                msg = Float64()
                msg.data = float(q[i])
                self.pubs[i].publish(msg)
            time.sleep(0.05)

    def calculate_and_move(self):
        self.get_logger().info("--> Calculating Trajectories...")

        # Reset Gripper (Open)
        self.set_gripper(False)

        boxB_pos = [0.5, -0.2, 0.0]
        boxA_pos = [0.3, 0.4, 0.0]
        R_down = SE3.Rx(np.pi)

        # Pick & Place Coords
        z_pick = boxB_pos[2] + 0.021
        T_Pick_Grasp = SE3(boxB_pos[0], boxB_pos[1], z_pick) * R_down
        T_Pick_Appr  = SE3(boxB_pos[0], boxB_pos[1], z_pick + 0.15) * R_down

        z_place = boxA_pos[2] + 0.14
        T_Place_Drop = SE3(boxA_pos[0], boxA_pos[1], z_place) * R_down
        T_Place_Appr = SE3(boxA_pos[0], boxA_pos[1], z_place + 0.15) * R_down

        q_home = np.array([0, 0, 0, 0, 0, 0])

        try:
            sol_pick_appr = self.gp7.ikine_LM(T_Pick_Appr, q0=q_home).q
            sol_pick      = self.gp7.ikine_LM(T_Pick_Grasp, q0=sol_pick_appr).q
            sol_place_appr= self.gp7.ikine_LM(T_Place_Appr, q0=sol_pick).q
            sol_place     = self.gp7.ikine_LM(T_Place_Drop, q0=sol_place_appr).q
        except:
            self.get_logger().error("IK Failed")
            return

        # Trajectories
        steps = 50
        traj1 = rtb.tools.trajectory.jtraj(q_home, sol_pick_appr, steps)
        traj2 = rtb.tools.trajectory.jtraj(sol_pick_appr, sol_pick, 40)
        traj3 = rtb.tools.trajectory.jtraj(sol_pick, sol_pick_appr, 40)
        traj4 = rtb.tools.trajectory.jtraj(sol_pick_appr, sol_place_appr, steps)
        traj5 = rtb.tools.trajectory.jtraj(sol_place_appr, sol_place, 40)
        traj6 = rtb.tools.trajectory.jtraj(sol_place, sol_place_appr, 40)
        traj7 = rtb.tools.trajectory.jtraj(sol_place_appr, q_home, steps)

        self.get_logger().info("--> Execution Start")
        
        self.move_segment(traj1.q) # Approach
        self.move_segment(traj2.q) # Descend
        
        # --- GRIPPER CLOSE ---
        time.sleep(0.5)
        self.set_gripper(True)
        time.sleep(1.0)
        
        self.move_segment(traj3.q) # Lift
        self.move_segment(traj4.q) # Transfer
        self.move_segment(traj5.q) # Place
        
        # --- GRIPPER OPEN ---
        time.sleep(0.5)
        self.set_gripper(False)
        time.sleep(1.0)
        
        self.move_segment(traj6.q) # Retreat
        self.move_segment(traj7.q) # Home
        
        self.get_logger().info("--> Done")

def main():
    rclpy.init()
    node = GP7Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
