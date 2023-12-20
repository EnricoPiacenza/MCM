function [theta,v] = ComputeInverseAngleAxis(R)
%N.B. this R should be a rotation matrix, namely the aRb passed in the main

%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % eig()
    % find()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.
    
    % Check matrix R to see if its size is 3x3
    if isequal(size(R), [3, 3])
        
        % Check matrix R to see if it is orthogonal
        %if A == eye(3)
        if isequaltol(R*R', eye(3), 0.0001)

            % Check matrix R to see if it is proper: det(R) = 1
            if isequaltol(det(R), 1, 0.0001)
                % Compute the angle of rotation
                theta = (acos((trace(R)-1)/2));

                if theta == 0
                    error('INFINITE SINGULARITY')
                elseif theta == pi
                    error('DUAL SINGULARITY')
                end

                % Calculate eigenvalues and eigenvectors of R
                [eigenvectors, eigenvalues] = eig(R);
                
                % Compute the axis of rotation
                for i=1:1:3
                    if  isequaltol(eigenvalues(i, i), 1, 0.0001)
                        for j=1:1:3
                            v(j) = eigenvectors(j, i);
                        end
                    end
                end

                orthogonal_vector = null(v(:).');
                orthogonal_vector = orthogonal_vector(:, 1);

                %check the sign of rotation to solve the ambiguity
                %the sign of rotation is that of the product 
                %vâ‹…(orthogonal_vector Ã— R*orthogonal_vector), where
                %orthogonal_vector is any vector orthogonal to v
                if dot(v, cross(orthogonal_vector, R*orthogonal_vector)) < 0
                    v = -v;
                end

            else
              error('DETERMINANT OF THE INPUT MATRIX IS NOT 1')
            end
        else
             error('NOT ORTHOGONAL INPUT MATRIX')
        end
    else
       error('WRONG SIZE OF THE INPUT MATRIX')
    end
end

