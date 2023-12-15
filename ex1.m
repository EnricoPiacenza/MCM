clc;
clear;
close all;
addpath('include');
%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix

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
J = GetJacobian(bTi(:,:,1:7),bTi(:,:,7),jointType);
