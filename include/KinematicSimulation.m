function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

% Updating q
q = q + ts * q_dot;

% Saturating the joint velocities
for i = 1 : size(q,1)
    if(q(i) < q_min(i))
        q(i) = q_min(i);
    elseif(q(i) > q_max(i))
        q(i) = q_max(i);
    end
end
end