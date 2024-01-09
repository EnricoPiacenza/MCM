function J = GetJacobian(biTei, bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% base to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

Ja = zeros(3,size(jointType,1));
Jl = zeros(3,size(jointType,1));

% Angular Jacobian
for i = 1 : size(jointType,1)

    if(jointType(i) == 0)   % Rotational
        Ja(:,i) = biTei(1:3,3,i);
    elseif(jointType(i) == 1)   % Prismatic
        Ja(:,i) = 0;
    else
        error("Invalid jointType");
    end
end


% Linear Jacobian
for i = 1 : size(jointType,1)

    if(jointType(i) == 0)   % Rotational
        irn = bTe(1:3,4) - biTei(1:3,4,i);
        Jl(:,i) = cross(biTei(1:3,3,i), irn);
    elseif(jointType(i) == 1)   % Prismatic
        Jl(:,i) = biTei(1:3,3,i);
    else
        error("Invalid jointType");
    end
end

J = [Ja;Jl];
end