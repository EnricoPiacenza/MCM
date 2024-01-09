%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');

%% 1.
%% Q1.1
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;                  % Number of manipulator's links.
JointType = [0 0 0 0 0 0 0];        % Boolean that specifies two possible link types: Rotational (0), Prismatic (1).
bri = zeros(3,numberOfLinks);       % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base

%% Q1.2

% Initial joint configuration 
q = [0,0,0,0,0,0,0];
q1 = [0,0,0,0,0,pi/2,0];
q2 = [0, pi/2, 0, -pi/2, 0, 0, 0];
q3 = [pi/4, pi/2, -pi/8, -pi/2, pi/4, 2/3*pi, 0];

% Return the model matrices for each joint configuration, from q to q3
iTj_q  = GetDirectGeometry(q,geom_model,JointType,numberOfLinks);
iTj_q1 = GetDirectGeometry(q1,geom_model,JointType,numberOfLinks);
iTj_q2 = GetDirectGeometry(q2,geom_model,JointType,numberOfLinks);
iTj_q3 = GetDirectGeometry(q3,geom_model,JointType,numberOfLinks);

%% Q1.3

for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q,i);
end

iTj = GetFrameWrtFrame(1,3,iTj_q);

for i = 1:numberOfLinks
    bri(:,i) = GetBasicVectorWrtBase(iTj_q,i);
end

%% Q1.4
%% First set
qi = [0,0,0,0,0,0,0];
qf = [pi/4, pi/2, -pi/8, -pi/2, pi/4, 2*pi/3, 0];
numberOfSteps = 30;

delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for i = 1:numberOfSteps

    % Get the initial position of all the joints of the robot
    iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

    for j = 1:numberOfLinks
        bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
        plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
    end

    % Set axis limits and equal aspect ratio
    axis equal;
    axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
    axis(axisLimits);
    grid on;

    % Set axis labels
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');

    pause(0.1);

    qi = qi + delta;
end
pause();

%% Second set
qi = [0, pi/2, 0, -pi/2, 0, 0, 0];
qf = [0,0,0,0,0,0,0];
delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for i = 1:numberOfSteps

    %get the initial position of all the joints of the robot
    iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

    for j = 1:numberOfLinks
        bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
        plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
    end
    % Set axis limits and equal aspect ratio
    axis equal;
    axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
    axis(axisLimits);
    grid on;

    % Set axis labels
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');

    pause(0.1);

    qi = qi + delta;
end
pause();
%% Third set

qi = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qf = [2, 2, 2, 2, 2, 2, 2];
delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for i = 1:numberOfSteps

    %get the initial position of all the joints of the robot
    iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

    for j = 1:numberOfLinks
        bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
        plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
    end
    % Set axis limits and equal aspect ratio
    axis equal;
    axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
    axis(axisLimits);
    grid on;

    % Set axis labels
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');

    pause(0.1);

    qi = qi + delta;
end
pause();
%% Q1.5
%% First set (modified)
qi = [0,0,0,0,0,0,0];
qf = [pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];

delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for k = 1:numberOfLinks
    for i = 1:numberOfSteps

        %get the initial position of all the joints of the robot
        iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

        for j = 1:numberOfLinks
            bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
            plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
        end
        % Set axis limits and equal aspect ratio
        axis equal;
        axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
        axis(axisLimits);
        grid on;

        % Set axis labels
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');

        pause(0.1);

        qi(k) = qi(k) + delta(k);
    end
end
pause();
%% Second set
qi = [0, pi/2, 0, -pi/2, 0, 0, 0];
qf = [0,0,0,0,0,0,0];

delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for k = 1:numberOfLinks
    for i = 1:numberOfSteps

        %get the initial position of all the joints of the robot
        iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

        for j = 1:numberOfLinks
            bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
            plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
        end
        % Set axis limits and equal aspect ratio
        axis equal;
        axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
        axis(axisLimits);
        grid on;

        % Set axis labels
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');

        pause(0.1);

        qi(k) = qi(k) + delta(k);
    end
end
pause();
%% Third set

qi = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qf = [2, 2, 2, 2, 2, 2, 2];

delta = (qf-qi)/numberOfSteps;
bri_i = zeros(3, numberOfLinks);

for k = 1:numberOfLinks
    for i = 1:numberOfSteps

        %get the initial position of all the joints of the robot
        iTj_qi = GetDirectGeometry(qi,geom_model,JointType,numberOfLinks);

        for j = 1:numberOfLinks
            bri_i(:,j) = GetBasicVectorWrtBase(iTj_qi,j);
            plot3(bri_i(1,:),bri_i(2,:),bri_i(3,:), '-o', 'LineWidth',2);
        end
        % Set axis limits and equal aspect ratio
        axis equal;
        axisLimits = max(abs(bri_i(:))) * [-1, 1, -1, 1, -1, 1]; % Adjust as needed
        axis(axisLimits);
        grid on;

        % Set axis labels
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');

        pause(0.1);

        qi(k) = qi(k) + delta(k);
    end
end
pause();