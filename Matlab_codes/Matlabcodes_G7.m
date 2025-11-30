%% ==============================================================================
%  Mini Project: Yaskawa GP7 Robot Simulation
%  Task 2: Motion Task (Pick Box B -> Place on Box A)
% ==============================================================================
clear; clc; close all;

%% ------------------------------------------------------------------------------
%  PART 1: ROBOT SETUP (D-H PARAMETERS)
%  Using 'Standard' D-H convention
% ------------------------------------------------------------------------------

% 1.1 Define Dimensions (meters) - From Manual [cite: 1103]
L1_d = 0.330;  L1_a = 0.040;
L2_a = 0.385;
L3_u = 0.340;
L4_d = 0.080;
Gripper_Len = 0.100; % Estimated gripper length [cite: 20]

% 1.2 Define Links
L(1) = Link([0, L1_d, L1_a, -pi/2]);
L(2) = Link([0, 0,    L2_a,  0    ]);
L(3) = Link([0, 0,    0.040, -pi/2]); % Elbow offset
L(4) = Link([0, L3_u, 0,     pi/2 ]);
L(5) = Link([0, 0,    0,     -pi/2]);
L(6) = Link([0, L4_d, 0,     0    ]);

% 1.3 Joint Limits [cite: 963]
L(1).qlim = deg2rad([-170 170]); L(2).qlim = deg2rad([-65 145]);
L(3).qlim = deg2rad([-70 190]);  L(4).qlim = deg2rad([-190 190]);
L(5).qlim = deg2rad([-135 135]); L(6).qlim = deg2rad([-360 360]);

% 1.4 Create Robot
gp7 = SerialLink(L, 'name', 'Yaskawa GP7');
gp7.tool = SE3(0, 0, Gripper_Len);

fprintf('--> Yaskawa GP7 Model Created.\n');

%% ------------------------------------------------------------------------------
%  PART 2: MOTION TASK SETUP
%  Requirement: Place Box B at center of top surface of Box A
% ------------------------------------------------------------------------------

% 2.1 Environment Definitions
% Box A: 30x30x20 cm at (0.3, 0.4, 0)
boxA_pos = [0.3, 0.4, 0.0]; 
boxA_dim = [0.3, 0.3, 0.2]; % Height = 0.2m

% Box B: 5x5x5 cm at random reachable location
boxB_pos = [0.5, -0.2, 0.0]; 
boxB_dim = [0.05, 0.05, 0.05]; % Height = 0.05m

% 2.2 Define "Upright" Orientation
% To keep the box upright, the gripper must point DOWN (-Z direction) at all times.
% Rotation of 180 deg (pi) around X-axis achieves this.
R_down = SE3.Rx(pi); 

% 2.3 Calculate Waypoints (Cartesian Poses)

% --- Waypoint 1: PICK (Box B) ---
% We grip the TOP of Box B.
% Z = Floor + BoxB_Height
z_pick = boxB_pos(3) + boxB_dim(3); 
T_Pick_Grasp = SE3(boxB_pos(1), boxB_pos(2), z_pick) * R_down;

% Approach point (15cm above Box B)
T_Pick_Appr = SE3(boxB_pos(1), boxB_pos(2), z_pick + 0.15) * R_down;

% --- Waypoint 2: PLACE (Box A) ---
% Target: "Center of the top surface of Box A" [cite: 30]
% Z = Floor + BoxA_Height + BoxB_Height
% (We add BoxB_Height because the gripper holds the *top* of Box B)
z_place = boxA_pos(3) + boxA_dim(3) + boxB_dim(3);
T_Place_Drop = SE3(boxA_pos(1), boxA_pos(2), z_place) * R_down;

% Approach point (15cm above Box A)
T_Place_Appr = SE3(boxA_pos(1), boxA_pos(2), z_place + 0.15) * R_down;

%% ------------------------------------------------------------------------------
%  PART 3: INVERSE KINEMATICS & TRAJECTORY GENERATION
% ------------------------------------------------------------------------------
fprintf('--> Calculating Trajectories...\n');

% 3.1 Home Position (Straight Up)
q_home = [0, -pi/2, pi/2, 0, 0, 0]; 
weights = [1 1 1 1 1 1]; % XYZ + RPY Priority

% 3.2 Solve IK for Key Points
% We use the previous configuration as 'q0' (seed) to ensure smooth motion
% and prevent the arm from "flipping" (changing configuration), maintaining orientation.

q_pick_appr  = gp7.ikine(T_Pick_Appr,  'q0', q_home,      'mask', weights);
q_pick_grasp = gp7.ikine(T_Pick_Grasp, 'q0', q_pick_appr, 'mask', weights);
q_place_appr = gp7.ikine(T_Place_Appr, 'q0', q_pick_grasp,'mask', weights);
q_place_drop = gp7.ikine(T_Place_Drop, 'q0', q_place_appr,'mask', weights);

% 3.3 Generate Trajectories (Smooth Velocity Profile) [cite: 41]
steps = 50; 

% 1. Move to Pre-Pick
traj1 = jtraj(q_home, q_pick_appr, steps);
% 2. Lower to Pick (Linear Vertical Move)
traj2 = jtraj(q_pick_appr, q_pick_grasp, steps/2);
% 3. Lift Box B (Vertical Retreat - Crucial for "Upright" stability)
traj3 = jtraj(q_pick_grasp, q_pick_appr, steps/2);
% 4. Transfer to Box A (Horizontal Move)
traj4 = jtraj(q_pick_appr, q_place_appr, steps);
% 5. Lower to Place (Linear Vertical Move)
traj5 = jtraj(q_place_appr, q_place_drop, steps/2);
% 6. Release & Retreat
traj6 = jtraj(q_place_drop, q_place_appr, steps/2);
% 7. Return Home
traj7 = jtraj(q_place_appr, q_home, steps);

% Combine all segments
full_trajectory = [traj1; traj2; traj3; traj4; traj5; traj6; traj7];

%% ------------------------------------------------------------------------------
%  PART 4: SIMULATION VISUALIZATION
% ------------------------------------------------------------------------------
fprintf('--> Starting Animation.\n');
figure('Name', 'Yaskawa GP7 - Motion Task', 'Color', 'w');

% Plot robot
% 'trail' shows the path of Box B
% 'floorlevel' sets the ground at z=0
gp7.plot(full_trajectory, 'trail', 'b-', 'workspace', [-0.5 1.0 -0.8 0.8 0 1.2], 'floorlevel', 0);

title('Task: Pick Box B -> Place Center Top of Box A');
fprintf('--> Simulation Complete.\n');