function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j,iTj_q)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% iTj_q: vector of matrices containing the transformation matrices from link i to link j for the current q.
% The size of biTei is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in iTj_q

    iTj = eye(4);
    
    if linkNumber_i < linkNumber_j
        for i = (linkNumber_i+1):1:linkNumber_j      
            iTj = iTj * iTj_q(:,:,i);
        end

    % Since the minus transpose for the transformation matrix coincide with
    % the inverse we use it
    elseif linkNumber_j < linkNumber_i
        min = linkNumber_j;
        max = linkNumber_i;
        min = min+1;
        %for i = linkNumber_i:-1:(linkNumber_i+1)  
        for i = max:-1:min
            iTj = iTj * inv(iTj_q(:,:,i));
        end

    else
        iTj = eye(4);
    end

end

