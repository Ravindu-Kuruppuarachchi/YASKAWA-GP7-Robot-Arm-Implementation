%% ==============================================================================
%  Mini Project: Yaskawa GP7 Robot Simulation
%  Task: Pick Box B -> Place on Box A (With Data Output)
% ==============================================================================
clear; clc; close all;
% Clear old definitions to prevent version conflicts
clear classes;

%% ------------------------------------------------------------------------------
%  PART 1: ROBOT SETUP (D-H PARAMETERS)
% ------------------------------------------------------------------------------
% 1.1 Define Physical Dimensions (meters)
L1_d = 0.330;  % Base height
L1_a = 0.040;  % Horizontal offset
L2_a = 0.445;  % Lower arm length
L3_u = 0.440;  % Upper arm length
L4_d = 0.440;  % Upper arm length (using d for vertical)
L6_d = 0.080;  % Wrist/Flange length
Gripper_Len = 0.14; 

% 1.2 Define Links
% Link 1: S-Axis
L(1) = Link();
L(1).d = L1_d; L(1).a = L1_a; L(1).alpha = -pi/2;

% Link 2: L-Axis (Lower Arm)
% OFFSET: Add pi/2 so q=0 is Vertical (Candle Pose)
L(2) = Link();
L(2).d = 0;    L(2).a = L2_a; L(2).alpha = 0;
L(2).offset = pi/2;

% Link 3: U-Axis (Upper Arm)
L(3) = Link();
L(3).d = 0;    L(3).a = 0.040; L(3).alpha = -pi/2;

% Link 4: R-Axis (Arm Roll)
L(4) = Link();
L(4).d = L3_u; L(4).a = 0;    L(4).alpha = pi/2;

% Link 5: B-Axis (Wrist Bend)
L(5) = Link();
L(5).d = 0;    L(5).a = 0;    L(5).alpha = -pi/2;

% Link 6: T-Axis (Tool Flange)
L(6) = Link();
L(6).d = L6_d; L(6).a = 0;    L(6).alpha = 0;

% 1.3 Joint Limits
L(1).qlim = deg2rad([-170 170]);
L(2).qlim = deg2rad([-65 145]);
L(3).qlim = deg2rad([-70 190]);
L(4).qlim = deg2rad([-190 190]);
L(5).qlim = deg2rad([-135 135]);
L(6).qlim = deg2rad([-360 360]);

% 1.4 Create Robot Object
gp7 = SerialLink(L, 'name', 'Yaskawa GP7');
gp7.tool = SE3(0, 0, Gripper_Len);
fprintf('--> Yaskawa GP7 Model Created.\n');

%% ------------------------------------------------------------------------------
%  PART 2: MOTION TASK SETUP
% ------------------------------------------------------------------------------
% 2.1 Environment Definitions
boxA_pos = [0.3, 0.4, 0.0]; 
boxA_dim = [0.3, 0.3, 0.2]; % LxWxH
boxB_pos = [0.5, -0.2, 0.0]; 
boxB_dim = [0.05, 0.05, 0.05]; % LxWxH

% 2.2 Define Orientation (End-effector Down)
R_down = SE3.Rx(pi); 

% 2.3 Calculate Waypoints
z_pick = boxB_pos(3) + boxB_dim(3); 
T_Pick_Grasp = SE3(boxB_pos(1), boxB_pos(2), z_pick) * R_down;
T_Pick_Appr  = SE3(boxB_pos(1), boxB_pos(2), z_pick + 0.15) * R_down;

z_place = boxA_pos(3) + boxA_dim(3); 
T_Place_Drop = SE3(boxA_pos(1), boxA_pos(2), z_place) * R_down;
T_Place_Appr = SE3(boxA_pos(1), boxA_pos(2), z_place + 0.15) * R_down;

%% ------------------------------------------------------------------------------
%  PART 3: TRAJECTORY GENERATION
% ------------------------------------------------------------------------------
fprintf('--> Calculating Trajectories...\n');
q_home = [0, -pi, 0, 0, 0, 0];  
weights = [1 1 1 1 1 1];

% 3.2 Inverse Kinematics
q_pick_appr  = gp7.ikine(T_Pick_Appr,  'q0', q_home,      'mask', weights);
q_pick_grasp = gp7.ikine(T_Pick_Grasp, 'q0', q_pick_appr, 'mask', weights);
q_place_appr = gp7.ikine(T_Place_Appr, 'q0', q_pick_grasp,'mask', weights);
q_place_drop = gp7.ikine(T_Place_Drop, 'q0', q_place_appr,'mask', weights);

% 3.3 Generate Segments
steps = 50; 
traj1 = jtraj(q_home, q_pick_appr, steps);       
traj2 = jtraj(q_pick_appr, q_pick_grasp, steps/2); 
traj3 = jtraj(q_pick_grasp, q_pick_appr, steps/2); 
traj4 = jtraj(q_pick_appr, q_place_appr, steps);   
traj5 = jtraj(q_place_appr, q_place_drop, steps/2);
traj6 = jtraj(q_place_drop, q_place_appr, steps/2);
traj7 = jtraj(q_place_appr, q_home, steps);        

full_trajectory = [traj1; traj2; traj3; traj4; traj5; traj6; traj7];

%% ------------------------------------------------------------------------------
%  PART 4: VISUALIZATION WITH BOXES
% ------------------------------------------------------------------------------
fprintf('--> Initializing Visualization...\n');
figure('Name', 'Yaskawa GP7 - Simulation', 'Color', 'w');
axis([-0.5 1.0 -0.8 0.8 0 1.2]); 
hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 4.1 Draw Static Box A (Red)
draw_cuboid(boxA_pos, boxA_dim, 'r', 0.3);

% 4.2 Initialize Box B (Blue) using hgtransform
boxB_transform = hgtransform; 
draw_cuboid([0,0,0], boxB_dim, 'b', 1, boxB_transform); 
M_initial = makehgtform('translate', [boxB_pos(1), boxB_pos(2), boxB_pos(3) + boxB_dim(3)/2]);
set(boxB_transform, 'Matrix', M_initial);

% 4.3 Initialize Robot Plot
gp7.plot(q_home, 'workspace', [-0.5 1.0 -0.8 0.8 0 1.2], 'floorlevel', 0);

% 4.4 Plot Static Coordinate Frames (RGB)
hold on;
T_accum = gp7.base;
for i = 1:gp7.n
    T_accum = T_accum * gp7.links(i).A(q_home(i));
    if isa(T_accum, 'SE3'), M_current = T_accum.T; else, M_current = T_accum; end
    trplot(M_current, 'length', 0.3, 'rgb', 'arrow', 'frame', num2str(i));
end

% 4.5 Define Simulation Indices
idx_grasp = size(traj1,1) + size(traj2,1); 
idx_drop  = idx_grasp + size(traj3,1) + size(traj4,1) + size(traj5,1);

fprintf('--> Starting Animation...\n');
pause(1);

% 4.6 Animation Loop
for i = 1:size(full_trajectory, 1)
    q_curr = full_trajectory(i, :);
    gp7.animate(q_curr);
    
    if i > idx_grasp && i <= idx_drop
        T_ee = gp7.fkine(q_curr);
        if isa(T_ee, 'SE3'), M_ee = T_ee.T; else, M_ee = T_ee; end
        
        offset_z = -boxB_dim(3)/2;
        M_offset = makehgtform('translate', [0, 0, offset_z]);
        set(boxB_transform, 'Matrix', M_ee * M_offset);
    end
    drawnow;
end
fprintf('--> Simulation Complete.\n');

%% ------------------------------------------------------------------------------
%  PART 5: DATA OUTPUT (DH TABLE & MATRICES)
% ------------------------------------------------------------------------------
fprintf('\n======================================================\n');
fprintf('             YASKAWA GP7 - KINEMATICS DATA            \n');
fprintf('======================================================\n');

% 5.1 PRINT DH TABLE
fprintf('\n--- 1. Denavit-Hartenberg (D-H) Table ---\n');
% Using the toolbox built-in display for cleanliness
gp7.display(); 

% 5.2 PRINT MATRICES (Using q_home configuration)
fprintf('\n--- 2. Forward Kinematics Matrices (at Home Pose) ---\n');
T_accum = gp7.base; % Initialize with Base Transform (usually Identity)

for i = 1:gp7.n
    % A) INDIVIDUAL JOINT MATRIX (from i-1 to i)
    % This is the transformation of Link(i) given the angle q_home(i)
    T_individual = gp7.links(i).A(q_home(i));
    
    % Check for SE3 vs Matrix type for printing
    if isa(T_individual, 'SE3'), M_ind = T_individual.T; else, M_ind = T_individual; end
    
    fprintf('\n[JOINT %d] Individual Transform (Link %d to %d):\n', i, i-1, i);
    disp(M_ind);
    
    % B) COMBINED MATRIX (from Base to i)
    % Accumulate the transform
    T_accum = T_accum * T_individual;
    
    if isa(T_accum, 'SE3'), M_acc = T_accum.T; else, M_acc = T_accum; end
    
    fprintf('[JOINT %d] Combined Transform (Base to Link %d):\n', i, i);
    disp(M_acc);
    fprintf('------------------------------------------------------\n');
end

% Print Final End Effector Position (including Tool)
fprintf('\n[End-Effector] Final Pose (including Tool):\n');
T_final = gp7.fkine(q_home);
if isa(T_final, 'SE3'), disp(T_final.T); else, disp(T_final); end

%% ------------------------------------------------------------------------------
%  HELPER FUNCTION: DRAW CUBOID
% ------------------------------------------------------------------------------
function draw_cuboid(pos, dim, color, alpha, parent_transform)
    if nargin < 5
        parent_transform = [];
    end
    x = dim(1); y = dim(2); z = dim(3);
    
    V = [ -x/2 -y/2 -z/2; x/2 -y/2 -z/2; x/2 y/2 -z/2; -x/2 y/2 -z/2; 
          -x/2 -y/2 z/2;  x/2 -y/2 z/2;  x/2 y/2 z/2;  -x/2 y/2 z/2]; 
      
    if isempty(parent_transform)
        V = V + [pos(1), pos(2), pos(3) + z/2];
    end
    F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    if isempty(parent_transform)
        patch('Vertices', V, 'Faces', F, 'FaceColor', color, 'FaceAlpha', alpha);
    else
        patch('Vertices', V, 'Faces', F, 'FaceColor', color, 'FaceAlpha', alpha, 'Parent', parent_transform);
    end
end