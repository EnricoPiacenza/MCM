clc;
clear;
close all;
addpath('include');
%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix

% The same model of assignment 2
% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
jointType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q0 = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
q1 =[1.8,1.8,1.8,1.8,1.8,1.8,1.8];
q2 =[0.3,1.4,0.1,2.0,0,1.3,0];
q3 =[0,0.1,0.2,0.3,0.4,0.5,0];
q4 = [1,1,1,1,1,1,1];

%% Compute direct geometry
iTj_q0  = GetDirectGeometry(q0,geom_model,jointType,numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q0,i);
end

% Computing end effector jacobian 
J0 = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);

%% Repeat for each configuration from q1 to q4
%q1
% Compute direct geometry
iTj_q1  = GetDirectGeometry(q1,geom_model,jointType,numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q1,i);
end

% Computing end effector jacobian 
J1 = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);

%q2
% Compute direct geometry
iTj_q2  = GetDirectGeometry(q2,geom_model,jointType,numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q2,i);
end

% Computing end effector jacobian 
J2 = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);

%q3
% Compute direct geometry
iTj_q3  = GetDirectGeometry(q3,geom_model,jointType,numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q3,i);
end

% Computing end effector jacobian 
J3 = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);

%q4
% Compute direct geometry
iTj_q4  = GetDirectGeometry(q4,geom_model,jointType,numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(iTj_q4,i);
end

% Computing end effector jacobian 
J4 = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);