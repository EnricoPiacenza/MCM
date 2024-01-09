function [rot_matrix] = quatToRot(q0,q1,q2,q3)
% quatToRot convert a quaternion into a rotation matrix
    %Covert a quaternion into a full three-dimensional rotation matrix.
 
    %Input
    %:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)
    %q0 is mu, q1,q2 and q3 are the components of epsilon
    epsilonx = zeros(3);

    epsilonx(2,1) = q3;
    epsilonx(3,1) = -q2;
    epsilonx(1,2) = -q3;
    epsilonx(3,2) = q1;
    epsilonx(1,3) = q2;
    epsilonx(2,3) = -q1;
 
    %Output
    %return: A 3x3 element matrix representing the full 3D rotation matrix. 

    %3x3 rotation matrix
    rot_matrix = eye(3) + 2 * q0 * epsilonx + 2 * epsilonx^2;

end