function iTj_q = DirectGeometry(qi, iTj, JointType_i)
% DirectGeometry Function 
% inputs: 
% qi : current joint position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>; 
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/traslation of the joint

if JointType_i == 0 % rotational
    iRj = iTj(1:3,1:3);
    Rz = [cos(qi) -sin(qi) 0; 
          sin(qi)  cos(qi) 0;
          0        0       1];
    % Rotational matrix changes
    iRj_q = iRj * Rz;
    % Vector r does not change (so it's equal to the one in iTj)
    iTj_q = iTj;
    iTj_q(1:3,1:3) = iRj_q; 

elseif JointType_i == 1 % prismatic
    irj = iTj(1:3,4);
    %r changes
    irj_q = irj + [0; 0; 1] * qi;
    % Rotational matrix does not change(so it's equal to the one in iTj)
    iTj_q = iTj;
    iTj_q(1:3,4) = irj_q;
end

end