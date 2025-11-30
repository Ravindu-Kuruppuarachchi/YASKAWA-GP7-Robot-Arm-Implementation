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

        # 1. Define Robot using D-H Parameters 
        # Dimensions (meters)
        L1_d = 0.330
        L1_a = 0.040
        L2_a = 0.445
        L3_u = 0.440
        L4_d = 0.440
        L6_d = 0.080
        Gripper_Len = 0.10

        # Define Links (RevoluteDH)
        L1 = rtb.RevoluteDH(d=L1_d, a=L1_a, alpha=-np.pi/2)
        # Link 2 includes the pi/2 offset
        L2 = rtb.RevoluteDH(d=0,    a=L2_a, alpha=0, offset=-np.pi/2)
        L3 = rtb.RevoluteDH(d=0,    a=0.040, alpha=-np.pi/2)
        L4 = rtb.RevoluteDH(d=L3_u, a=0,     alpha=np.pi/2)
        L5 = rtb.RevoluteDH(d=0,    a=0,     alpha=-np.pi/2)
        L6 = rtb.RevoluteDH(d=L6_d, a=0,     alpha=0)

        # Create Robot
        self.gp7 = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='Yaskawa GP7')
        self.gp7.tool = SE3(0, 0, Gripper_Len)

        self.get_logger().info("--> Yaskawa GP7 Model Created.")
        print(self.gp7)

        # 2. Setup ROS 2 Publishers 
        self.pubs = [
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_s/cmd_pos', 10),
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_l/cmd_pos', 10),
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_u/cmd_pos', 10),
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_r/cmd_pos', 10),
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_b/cmd_pos', 10),
            self.create_publisher(Float64, '/model/gp7_robot/joint/joint_t/cmd_pos', 10)
        ]

        # 3. Execute Motion Task 
        self.timer = self.create_timer(1.0, self.execute_trajectory_once)
        self.motion_started = False

    def execute_trajectory_once(self):
        if self.motion_started:
            return
        self.motion_started = True
        self.calculate_and_move()

    def calculate_and_move(self):
        self.get_logger().info("--> Calculating Trajectories...")

        # Environment Definitions 
        boxA_pos = [0.3, 0.4, 0.0]
        boxB_pos = [0.5, -0.2, 0.0]

        # Orientation (Downwards)
        R_down = SE3.Rx(np.pi)

        # Waypoints
        # Pick (Box B)
        z_pick = boxB_pos[2] + 0.05
        T_Pick_Grasp = SE3(boxB_pos[0], boxB_pos[1], z_pick) * R_down
        T_Pick_Appr  = SE3(boxB_pos[0], boxB_pos[1], z_pick + 0.15) * R_down

        # Place (Box A)
        z_place = boxA_pos[2] + 0.2 
        T_Place_Drop = SE3(boxA_pos[0], boxA_pos[1], z_place) * R_down
        T_Place_Appr = SE3(boxA_pos[0], boxA_pos[1], z_place + 0.15) * R_down

        # Inverse Kinematics
        q_home = np.array([0, 0, 0, 0, 0, 0])


        # Solve IK
        sol_pick_appr  = self.gp7.ikine_LM(T_Pick_Appr,  q0=q_home)
        sol_pick_grasp = self.gp7.ikine_LM(T_Pick_Grasp, q0=sol_pick_appr.q)
        sol_place_appr = self.gp7.ikine_LM(T_Place_Appr, q0=sol_pick_grasp.q)
        sol_place_drop = self.gp7.ikine_LM(T_Place_Drop, q0=sol_place_appr.q)

        if not sol_pick_appr.success:
            self.get_logger().error("IK Failed: Pick Approach")

        # Trajectory Segments 
        steps = 50
        traj1 = rtb.tools.trajectory.jtraj(q_home, sol_pick_appr.q, steps)
        traj2 = rtb.tools.trajectory.jtraj(sol_pick_appr.q, sol_pick_grasp.q, int(steps/2))
        traj3 = rtb.tools.trajectory.jtraj(sol_pick_grasp.q, sol_pick_appr.q, int(steps/2))
        traj4 = rtb.tools.trajectory.jtraj(sol_pick_appr.q, sol_place_appr.q, steps)
        traj5 = rtb.tools.trajectory.jtraj(sol_place_appr.q, sol_place_drop.q, int(steps/2))
        traj6 = rtb.tools.trajectory.jtraj(sol_place_drop.q, sol_place_appr.q, int(steps/2))
        traj7 = rtb.tools.trajectory.jtraj(sol_place_appr.q, q_home, steps)

        full_traj = np.concatenate([
            traj1.q, traj2.q, traj3.q, traj4.q, traj5.q, traj6.q, traj7.q
        ])

        self.get_logger().info(f"--> Trajectory generated: {len(full_traj)} steps. Starting execution...")

        # Send Commands to Gazebo 
        for q in full_traj:
            for i in range(6):
                msg = Float64()
                msg.data = float(q[i])
                self.pubs[i].publish(msg)
            
            time.sleep(0.05) 

        self.get_logger().info("--> Motion Task Complete.")

def main(args=None):
    rclpy.init(args=args)
    controller = GP7Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
