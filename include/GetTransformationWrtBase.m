function [bTi] = GetTransformationWrtBase(iTj_q, linkNumber)
%%% GetTransformatioWrtBase function
% inputs :
% iTj_q vector of matrices containing the transformation matrices from link i to link i +1 for the current joint position q.
% The size of iTj_q is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix
% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by iTj_q

    bTi = eye(4);
    
    for i = 1:1:linkNumber
        bTi = bTi * iTj_q(:,:,i);
    end
end