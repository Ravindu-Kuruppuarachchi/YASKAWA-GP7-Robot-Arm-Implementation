#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import time

class GP7Controller(Node):
    def __init__(self):
        super().__init__('gp7_controller')

        # Robot Setup 
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

        # Publishers 
        self.pubs = [
            self.create_publisher(Float64, f'/model/gp7_robot/joint/joint_{name}/cmd_pos', 10)
            for name in ['s', 'l', 'u', 'r', 'b', 't']
        ]
        self.gripper_pub = self.create_publisher(Bool, '/gripper/attach', 10)
        self.finger_l = self.create_publisher(Float64, '/model/gp7_robot/joint/finger_joint/cmd_pos', 10)
        self.finger_r = self.create_publisher(Float64, '/model/gp7_robot/joint/right_outer_knuckle_joint/cmd_pos', 10)

        # Timer
        self.timer = self.create_timer(1.0, self.execute_motion)
        self.motion_started = False

    def execute_motion(self):
        if self.motion_started: return
        self.motion_started = True
        self.calculate_and_move()

    def set_gripper(self, closed=False):

        # 1. Move Fingers
        pos = 0.9 if closed else 0.0
        msg_float = Float64()
        msg_float.data = pos
        
        # 2. Activate Plugin
        msg_bool = Bool()
        msg_bool.data = closed

        action = "CLOSING" if closed else "OPENING"
        self.get_logger().info(f"--> GRIPPER: {action}")

        for _ in range(5):
            self.finger_l.publish(msg_float)
            self.finger_r.publish(msg_float)
            self.gripper_pub.publish(msg_bool)
            time.sleep(0.02)

    def move_segment(self, trajectory):
        """Execute  pre-calculated trajectory array"""
        for q in trajectory:
            for i in range(6):
                msg = Float64()
                msg.data = float(q[i])
                self.pubs[i].publish(msg)
            time.sleep(0.1) # dt = 0.1

    def generate_trapezoidal(self, q_start, q_end, steps, dt=0.05):
        """Trapezoidal Velocity Profile Generator"""

        traj = []
        T = steps * dt
        t_acc = T / 3.0  # Acceleration time
        
        # Time vector
        times = np.linspace(0, T, steps)

        for t in times:
            q_t = np.zeros(6)
            for i in range(6):
                dist = q_end[i] - q_start[i]
                if abs(dist) < 1e-6:
                    q_t[i] = q_end[i]
                    continue
                
                # Calculate velocities and acceleration
                v_avg = dist / T
                v_max = 1.5 * v_avg # Peak velocity
                acc = v_max / t_acc

                # Profile Logic
                if t <= t_acc:
                    # Phase 1: Acceleration
                    q_t[i] = q_start[i] + 0.5 * acc * t**2
                elif t <= (T - t_acc):
                    # Phase 2: Constant Velocity
                    q_start_cruise = q_start[i] + 0.5 * acc * t_acc**2
                    q_t[i] = q_start_cruise + v_max * (t - t_acc)
                else:
                    # Phase 3: Deceleration
                    t_dec = t - (T - t_acc) # Time spent in decel phase
                    q_end_cruise = q_start[i] + 0.5 * acc * t_acc**2 + v_max * (T - 2*t_acc)
                    q_t[i] = q_end_cruise + v_max * t_dec - 0.5 * acc * t_dec**2
            
            traj.append(q_t)
        
        return np.array(traj)
        
    def log_waypoint(self, name, q):
        """Prints FK and IK for a specific configuration"""
        T = self.gp7.fkine(q)
        np.set_printoptions(precision=3, suppress=True)
        print("-"*60)
        print(f" COMPLETED MOTION: {name}")
        print(f"Current Joint Angles (IK Result):\n{q}")
        print(f"Current End-Effector Pose (FK Matrix):\n{T}")
        print("-"*60 + "\n")

    def calculate_and_move(self):
        self.get_logger().info("> Calculating Trajectories (Trapezoidal)")

        # Reset Gripper
        self.set_gripper(False)

        # Environment
        boxA_pos = [0.3, 0.4, 0.0]
        boxB_pos = [0.5, -0.2, 0.0]
        R_down = SE3.Rx(np.pi)

        # Coords
        z_pick = boxB_pos[2] + 0.021
        T_Pick_Grasp = SE3(boxB_pos[0], boxB_pos[1], z_pick) * R_down
        T_Pick_Appr  = SE3(boxB_pos[0], boxB_pos[1], z_pick + 0.15) * R_down

        z_place = boxA_pos[2] + 0.15
        T_Place_Drop = SE3(boxA_pos[0], boxA_pos[1], z_place) * R_down
        T_Place_Appr = SE3(boxA_pos[0], boxA_pos[1], z_place + 0.15) * R_down

        q_home = np.array([0, 0, 0, 0, 0, 0])

        # IK
        try:
            sol_pick_appr = self.gp7.ikine_LM(T_Pick_Appr, q0=q_home).q
            sol_pick      = self.gp7.ikine_LM(T_Pick_Grasp, q0=sol_pick_appr).q
            sol_place_appr= self.gp7.ikine_LM(T_Place_Appr, q0=sol_pick).q
            sol_place     = self.gp7.ikine_LM(T_Place_Drop, q0=sol_place_appr).q
        except:
            self.get_logger().error("IK Failed")
            return

        # Generate Trajectories using Custom Function
        steps = 50
        traj1 = self.generate_trapezoidal(q_home, sol_pick_appr, steps)
        traj2 = self.generate_trapezoidal(sol_pick_appr, sol_pick, 40)
        traj3 = self.generate_trapezoidal(sol_pick, sol_pick_appr, 40)
        traj4 = self.generate_trapezoidal(sol_pick_appr, sol_place_appr, steps)
        traj5 = self.generate_trapezoidal(sol_place_appr, sol_place, 40)
        traj6 = self.generate_trapezoidal(sol_place, sol_place_appr, 40)
        traj7 = self.generate_trapezoidal(sol_place_appr, q_home, steps)

        self.get_logger().info("--> Execution Start")
        
        self.move_segment(traj1) # Approach
        self.log_waypoint("PICK APPROACH", sol_pick_appr) # Print FK/IK
        self.move_segment(traj2) # Descent
        self.log_waypoint("PICK GRASP LOCATION", sol_pick) # Print FK/IK
        
        # Grasp
        time.sleep(0.5)
        self.set_gripper(True)
        time.sleep(1.4)
        
        self.move_segment(traj3) # Lift
        self.log_waypoint("LIFTED (RETREAT)", sol_pick_appr) # Print FK/IK
        self.move_segment(traj4) # Transfer
        self.log_waypoint("PLACE APPROACH", sol_place_appr) # Print FK/IK
        self.move_segment(traj5) # Place
        self.log_waypoint("PLACE DROP LOCATION", sol_place) # Print FK/IK
        
        
        # Release
        time.sleep(0.5)
        self.set_gripper(False)
        time.sleep(1.5)
        
        self.move_segment(traj6) # Retreat
        self.log_waypoint("RETREATED FROM PLACE", sol_place_appr) # Print FK/IK
        self.move_segment(traj7) # Home
        self.log_waypoint("RETURNED HOME", q_home) # Print FK/IK
        
        self.get_logger().info("--> Done")

def main():
    rclpy.init()
    node = GP7Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
