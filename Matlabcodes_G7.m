%% ==============================================================================
%  Mini Project: Yaskawa GP7 Robot Simulation (Kinematics & Control)
%  Combined Task 1 (Modeling) & Task 2 (Motion Planning)
% ==============================================================================
clear; clc; close all;

%% ------------------------------------------------------------------------------
%  PART 1: ROBOT MODELING (D-H PARAMETERS)
%  Dimensions derived from Yaskawa GP7 Manual (Fig 5-3b)
% ------------------------------------------------------------------------------

% 1.1 Define Physical Dimensions (meters)
L1_Base_Height      = 0.330;  % Vertical distance from base to Joint 2
L1_Horizontal_Off   = 0.040;  % Horizontal offset from Joint 1 to Joint 2
L2_Lower_Arm        = 0.445;  % Length of the Lower Arm (L-axis)
L3_Upper_Arm        = 0.440;  % Length of the Upper Arm (U-axis)
L4_Wrist_Flange     = 0.080;  % Distance from Wrist Center to Flange
Gripper_Length      = 0.100;  % Estimated length of simplified gripper

% 1.2 Create D-H Links
% Standard D-H convention: Link([theta, d, a, alpha])
% 'q' in the theta position indicates a revolute joint variable
L(1) = Link([0,      L1_Base_Height,    L1_Horizontal_Off,  -pi/2], 'standard');
L(2) = Link([0,      0,                 L2_Lower_Arm,       0    ], 'standard');
L(3) = Link([0,      0,                 0.040,              -pi/2], 'standard'); % Small structural offset
L(4) = Link([0,      L3_Upper_Arm,      0,                  pi/2 ], 'standard');
L(5) = Link([0,      0,                 0,                  -pi/2], 'standard');
L(6) = Link([0,      L4_Wrist_Flange,   0,                  0    ], 'standard');

% 1.3 Set Joint Limits (from Manual Table 5-2)
L(1).qlim = deg2rad([-170 170]); % S-axis
L(2).qlim = deg2rad([-65  145]); % L-axis
L(3).qlim = deg2rad([-70  190]); % U-axis
L(4).qlim = deg2rad([-190 190]); % R-axis
L(5).qlim = deg2rad([-135 135]); % B-axis
L(6).qlim = deg2rad([-360 360]); % T-axis

% 1.4 Instantiate Robot Object
gp7 = SerialLink(L, 'name', 'Yaskawa GP7');

% 1.5 Define Tool Transform (End-Effector)
% Adds the gripper length to the flange
gp7.tool = SE3(0, 0, Gripper_Length);

fprintf('--> Yaskawa GP7 Robot Model initialized.\n');
% gp7.display(); % Uncomment to see D-H table

%% ------------------------------------------------------------------------------
%  PART 2: KINEMATICS VERIFICATION
% ------------------------------------------------------------------------------

% 2.1 Define "Home" Configuration
% Typical industrial home: L-shape
q_home = [0, 0, 0, 0, -pi/2, 0]; 

% 2.2 Forward Kinematics (FK) Check
T_home = gp7.fkine(q_home);
fprintf('\n[FK Verification] End-Effector Pose at Home:\n');
disp(T_home.T);

% 2.3 Inverse Kinematics (IK) Check
% Using numerical solver with weighting mask [x y z roll pitch yaw]
weights = [1 1 1 1 1 1]; 
q_inv_check = gp7.ikine(T_home, 'q0', q_home, 'mask', weights);

% Calculate error
err_norm = norm(q_home - q_inv_check);
fprintf('[IK Verification] Error between original and recovered angles: %.6f rad\n', err_norm);

%% ------------------------------------------------------------------------------
%  PART 3: TRAJECTORY PLANNING (PICK AND PLACE)
%  Task: Move Box B to top of Box A, keeping orientation upright.
% ------------------------------------------------------------------------------

% 3.1 Define Environment Coordinates
% Box A (Target): Fixed at (0.3, 0.4, 0)
% Box B (Source): Random reachable location
% Dimensions: Box A (Height 0.2m), Box B (Height 0.05m)

boxA_pos = [0.3, 0.4, 0.0]; 
boxA_dim = [0.3, 0.3, 0.2]; % LxWxH

boxB_pos = [0.5, -0.2, 0.0]; 
boxB_dim = [0.05, 0.05, 0.05]; % LxWxH

% 3.2 Define Key Waypoints (Cartesian)
% NOTE: Z-coordinates must account for box height so we place ON TOP, not inside.

% -- Pick Position (Center of Box B) --
% Target Z = Floor + BoxB_Height
pick_point = [boxB_pos(1), boxB_pos(2), boxB_dim(3)];

% -- Place Position (Center Top of Box A) --
% Target Z = Floor + BoxA_Height
place_point = [boxA_pos(1), boxA_pos(2), boxA_dim(3)];

% 3.3 Define Orientation
% "Object must remain upright" -> End-effector points DOWN (-Z direction)
% Rotate 180 degrees (pi) around X-axis
R_down = SE3.Rx(pi);

% 3.4 Generate Full Motion Sequence (Poses)
% We add offsets (0.15m) for Approach/Retreat to avoid collisions

% 1. Pre-Pick (Approach)
T_Pick_Appr = SE3(pick_point + [0, 0, 0.15]) * R_down;

% 2. Pick (Grasp)
T_Pick_Grasp = SE3(pick_point) * R_down;

% 3. Pre-Place (Approach/Retreat) - Lift high enough to clear Box A
T_Place_Appr = SE3(place_point + [0, 0, 0.15]) * R_down;

% 4. Place (Drop) - On top of Box A
% Optional: Add half Box B height to place it *on* surface, not *in* surface
place_height_adjusted = place_point + [0, 0, boxB_dim(3)]; 
T_Place_Drop = SE3(place_height_adjusted) * R_down;

%% ------------------------------------------------------------------------------
%  PART 4: INVERSE KINEMATICS & TRAJECTORY GENERATION
% ------------------------------------------------------------------------------
fprintf('\n--> Generating Trajectories...\n');

% 4.1 Solve IK for all Key Waypoints
% Use 'q_home' as the initial guess to ensure consistent configuration
q_pick_appr  = gp7.ikine(T_Pick_Appr,  'q0', q_home,      'mask', weights);
q_pick_grasp = gp7.ikine(T_Pick_Grasp, 'q0', q_pick_appr, 'mask', weights);
q_place_appr = gp7.ikine(T_Place_Appr, 'q0', q_pick_grasp,'mask', weights);
q_place_drop = gp7.ikine(T_Place_Drop, 'q0', q_place_appr,'mask', weights);

% 4.2 Generate Smooth Joint Trajectories (Quintic Polynomial)
steps = 50; % Time steps per segment

% Segment 1: Home -> Pick Approach
traj1 = jtraj(q_home, q_pick_appr, steps);

% Segment 2: Approach -> Grasp (Linear descent)
traj2 = jtraj(q_pick_appr, q_pick_grasp, steps);

% Segment 3: Grasp -> Retreat (Lift box straight up)
traj3 = jtraj(q_pick_grasp, q_pick_appr, steps);

% Segment 4: Transfer (Move to Box A location)
traj4 = jtraj(q_pick_appr, q_place_appr, steps);

% Segment 5: Approach -> Drop (Lower box onto Box A)
traj5 = jtraj(q_place_appr, q_place_drop, steps);

% Segment 6: Drop -> Retreat (Release and move up)
traj6 = jtraj(q_place_drop, q_place_appr, steps);

% Segment 7: Return Home
traj7 = jtraj(q_place_appr, q_home, steps);

% Combine all segments into one matrix
full_trajectory = [traj1; traj2; traj3; traj4; traj5; traj6; traj7];

%% ------------------------------------------------------------------------------
%  PART 5: ANIMATION
% ------------------------------------------------------------------------------
fprintf('--> Starting Animation.\n');

figure('Name', 'Yaskawa GP7 Pick and Place Simulation', 'Color', 'w');

% Plot the robot
% 'trail' leaves a line showing the path of the end-effector
% 'workspace' sets the axes limits [xmin xmax ymin ymax zmin zmax]
gp7.plot(full_trajectory, 'trail', 'r-', 'workspace', [-0.5 1.2 -0.8 0.8 0 1.2], 'floorlevel', 0);

title('Simulation: Pick Box B -> Place on Box A');
fprintf('--> Simulation Complete.\n');