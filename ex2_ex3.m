%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
clear; clc; close all;
addpath('include')
model = load("panda.mat"); % don't worry about eventual warnings!
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Function that gives the transformation from <base> to <e-e>, given a
% configuration of the manipulator
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7');%DO NOT EDIT 

% Tool frame definition
phi = -44.98 * pi/180;
eOt = [0, 0, 0.2104]';
eRt = [cos(phi) -sin(phi)  0;
       sin(phi)  cos(phi)  0;
         0         0       1];

eTt = [eRt eOt];
eTt = [eTt; 0 0 0 1];

bTt = bTe * eTt;
bRt = bTt(1:3,1:3);
%% Goal definition 

% Goal frame rotation around y-axis of the e.e. initial configuration
alpha = pi/6;
bOg = [0.55; -0.3; 0.2]; % Meters

R_y = [cos(alpha),  0, sin(alpha);
           0,       1,      0    ;
       -sin(alpha), 0, cos(alpha)];

% Switch between the two cases (with and without the tool frame)
tool = false; % change to true for using the tool
if tool == true
    bRg = bRt * R_y;
    bTg = [bRg bOg];
    bTg = [bTg; 0 0 0 1];
else
    bRe = bTe(1:3,1:3);
    % controlling the ee frame
    % transformation matrix of goal frame w.r.t. base
    bRg =  bRe * R_y ;
    bTg = [bRg bOg];
    bTg = [bTg; 0 0 0 1];
end   

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 
% Start the inverse kinematic control  
q = q_init;

%% Simulation Loop
for i = t
    
    if tool == true %compute the error between the tool frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT

        bRe = bTe(1:3,1:3);
        bOe = bTe(1:3,4);

        % Computing rigid Jacobian
        b_ert = bRe * eOt;
        b_ertx = zeros(3,3);

        % Define the [vx] matrix values with v coordinates
        b_ertx(2,1) = b_ert(3);
        b_ertx(3,1) = -b_ert(2);
        b_ertx(1,2) = -b_ert(3);
        b_ertx(3,2) = b_ert(1);
        b_ertx(1,3) = b_ert(2);
        b_ertx(2,3) = -b_ert(1);

        % Rigid jacobian
        R_Jacobian = zeros(6,6);
        R_Jacobian(1:3,1:3) = eye(3);
        R_Jacobian(4:6,1:3) = b_ertx';
        R_Jacobian(4:6,4:6) = eye(3);

        % Jacobian
        bJt = R_Jacobian * bJe;

        bTt = bTe * eTt;   %QUESTO é QUELLO CHE HO CAMBIATO
        bRt = bTt(1:3,1:3);

        % Linear error
        tTg = pinv(bTt) * bTg;
        lin_err = bRt * tTg(1:3,4);

        % Angular error
        tRg = tTg(1:3, 1:3);
        [theta, v] = ComputeInverseAngleAxis(tRg);
        % calculate the angular error by projecting it on the base frame
        ang_err = bRt * (theta * v)';

        bOt = bOe + b_ert;   %ANCHE QUESTO L'HO SPOSTATO IN FONDO, SEMBRA FUNZIONARE
        % Updating bTt because the plot needs it
        bTt(1:3,1:3) = bRt;
        bTt(1:3,4) = bOt;

        Jacobian = bJt;
    else 
        % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        % lin_err = ...
        % ang_err = ...
        % cartesian error matrix between goal and ee, so eTg
        eTg = pinv(bTe) * bTg;
        
        % extract the rotation matrix bRe to use it for projection on the
        % base frame
        bRe = bTe(1:3,1:3);
    
        % extract the linear error from eTg and project it on the base
        % frame
        lin_err = bRe * eTg(1:3, 4);
    
        % extract the rotation error from eTg and convert it into angle-axis
        % representation
        eRg = eTg(1:3, 1:3);
        [theta, v] = ComputeInverseAngleAxis(eRg);
        % calculate the angular error by projecting it on the base frame
        ang_err = bRe * (theta * v)';
        Jacobian = tmp;
    end
    
    %% Compute the reference velocities
    % considering the fact the the goal does not move, so its velocities
    % are 
    b_Omega_e0_des = angular_gain * ang_err;
    b_v_e0_des = linear_gain * lin_err;
   
    %% Compute desired joint velocities
    x_dot = [b_Omega_e0_des; b_v_e0_des];
    q_dot = pinv(Jacobian) * x_dot;
    
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q, q_dot, ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    %switch visuals to off for seeing only the frames
    show(model.franka,[q',0,0],'visuals','on');
    hold on
    if tool == true
        %set the window size of the figure to "full-screen" for a better visualization
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    end
    hold off
    drawnow
    if(norm(x_dot) < 0.001)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
end