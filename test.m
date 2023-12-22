%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
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

% Goal definition 
bOg = [0.55 -0.3 0.2]';
theta = pi/6;
R_y = [cos(theta) 0     sin(theta);
       0          1     0;
       -sin(theta) 0    cos(theta)];

bOe = bTe(1:3,4);
% Cartesian error without tool
eOg = bOg - bOe;


phi = -44.98*pi/180; % angle phi defined in degrees converted in radians

% Tool frame definition
eOt = [0, 0, 0.2104]'; %  converted to meters

eRt = [cos(phi) -sin(phi) 0;
       sin(phi)  cos(phi) 0;
       0         0        1];
eTt = zeros(4,4);
eTt(1:3,1:3) = eRt; 
eTt(1:3,4) = eOt;
eTt(:,4) = 1;


bRe = bTe(1:3,1:3);
bRt = bRe*eRt;
bOt = bOe - eOt;
bTt = zeros(4,4);
bTt(1:3,1:3) = bRt;
bTt(1:3,4) = bOt;
bTt(4,4) = 1;


% Switch between the two cases (with and without the tool frame)
tool = true; % change to true for using the tool
if tool == true
    % The rotation matrix bRg is defined differently from the one without
    % tool.
    bRg = bRt*R_y;
    
    % Assembling  bTg
    bTg = zeros(4,4);
    bTg(1:3,1:3) = bRg;
    bTg(1:3,4) = bOg;
    bTg(4,4) = 1;
    
else
    % Defining bRg
    bRg = bRe*R_y;

    % Assembling  bTg
    bTg = zeros(4,4);
    bTg(1:3,1:3) = bRg;
    bTg(1:3,4) = bOg;
    bTg(4,4) = 1;
end   

% Control Proportional Gain 
% alpha = 0.2 is a vector containing both linear and angular gain. We
% decided to multiply independently those two quantities.
angular_gain = 0.2;
linear_gain = 0.2;

% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 

% Start the inverse kinematic control  
q = q_init;
% This is just a initialization. It is not needed but just shows how
% b_nu_e0 is computed
v_e0 = linear_gain * eOg; % the goal is not moving, so I do not have the feed forward term
rho = v_e0 * theta;
w_e0 = angular_gain * rho;

b_nu_e0 = [w_e0; v_e0];

J = geometricJacobian(model.franka,[q',0,0],'panda_link7');

J_inv = pinv(J);
q_dot = J_inv*b_nu_e0;

%% Simulation Loop
for i = t
    
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        bJt = tmp;
        % eJt = bJt(:,8:9);
        
        bRe = bTe(1:3,1:3);
        bOe = bTe(1:3,4);
        bRt = bRe*eRt;
        
        % Angular error
        tTg = pinv(bTt) * bTg;
        tRg = tTg(1:3, 1:3);
        [theta, v] = ComputeInverseAngleAxis(tRg);
        % calculate the angular error by projecting it on the base frame
        ang_err = bRt * (theta * v)';

        % Computing distance from frame <e> to <t> w.r.t. <b>
        eOt_b = bRe*eOt;
        
        % Defining the skew-symmetric matrix that will be needed by the
        % rigid Jacobian
        ert_x = [0,      -eOt_b(3)     eOt_b(2);
                 eOt_b(3)    0        -eOt_b(1);
                -eOt_b(2)   eOt_b(1)      0];

        % Computing bOt now that I have eOt_b
        bOt = bOe + eOt_b;
        
        % Updating bTt because the plot needs it
        bTt(1:3,1:3) = bRt;
        bTt(1:3,4) = bOt;

        % Defining the rigid Jacobian
        tSe = zeros(6,6);
        tSe(1:3,1:3) = eye(3);
        tSe(4:6,4:6) = eye(3);
        tSe(4:6,1:3) = ert_x'; 

        % Defining the Jacobian
        Jacobian = tSe*bJe;

        % Computing tOg, that is the linear error
        tOg = bOg - bOt;
        lin_err = tOg;
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        bRe = bTe(1:3,1:3);
        bOe = bTe(1:3,4);

        % Cartesian error
        eOg = bOg - bOe;
        lin_err = eOg;

        % Updating theta with ComputeInverseAngleAxis function
        [theta,v] = ComputeInverseAngleAxis(bRe'*bRg);

        % Computing angular error
        rho = bRe*(theta*v)';
        ang_err = rho;

        % Extracting the Jacobian
        Jacobian = tmp;
    end
    
       
    %% Compute the reference velocities
    % Computing the linear and the angular velocities with the linear and
    % angular error previously computed
    % The goal is not moving, so I do not have the feed forward term
    v_e0 = linear_gain * lin_err; 
    w_e0 = angular_gain * ang_err;
    
    % Collecting the velocities just computed in nu vector
    b_nu_e0 = [w_e0; v_e0];

    % Setting x_dot
    x_dot = b_nu_e0;

    %% Compute desired joint velocities 
    % Calculating the inverse of the Jacobian with pinv(), because it is
    % not a square matrix, so it is an allocation problem
    J_inv = pinv(Jacobian);
    
    % Computing q_dot
    q_dot = J_inv*b_nu_e0;

    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot(1:7),ts, qmin, qmax);
    
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