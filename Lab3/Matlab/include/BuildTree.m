function [iTj] = BuildTree()
% This function should build the tree of frames for the chosen manipulator.
% Inputs: 'None'
% Outputs: The tree of frames.

% iTj is a 3-dimensional matlab matrix, suitable for defining tree of
% frames. iTj should represent the transformation matrix between the i-th and j-th
% frames. iTj(row,col,link_idx)

% 0T1
iTj(1,1,1) = 1; iTj(1,2,1) = 0; iTj(1,3,1) = 0; iTj(1,4,1) = 0;
iTj(2,1,1) = 0; iTj(2,2,1) = 1; iTj(2,3,1) = 0; iTj(2,4,1) = 0;
iTj(3,1,1) = 0; iTj(3,2,1) = 0; iTj(3,3,1) = 1; iTj(3,4,1) = 0.175;
iTj(4,1,1) = 0; iTj(4,2,1) = 0; iTj(4,3,1) = 0; iTj(4,4,1) = 1;

%1T2
iTj(1,1,2) = -1; iTj(1,2,2) = 0; iTj(1,3,2) = 0;  iTj(1,4,2) = 0;
iTj(2,1,2) = 0; iTj(2,2,2) = 0; iTj(2,3,2) = 1;  iTj(2,4,2) = 0;
iTj(3,1,2) = 0; iTj(3,2,2) = 1; iTj(3,3,2) = 0;  iTj(3,4,2) = 0.098;
iTj(4,1,2) = 0; iTj(4,2,2) = 0; iTj(4,3,2) = 0;  iTj(4,4,2) = 1;

%2T3
iTj(1,1,3) = 0; iTj(1,2,3) = 0; iTj(1,3,3) = 1;  iTj(1,4,3) = 0.105;
iTj(2,1,3) = 1; iTj(2,2,3) = 0; iTj(2,3,3) = 0;  iTj(2,4,3) = 0;
iTj(3,1,3) = 0; iTj(3,2,3) = 1; iTj(3,3,3) = 0;  iTj(3,4,3) = 0;
iTj(4,1,3) = 0; iTj(4,2,3) = 0; iTj(4,3,3) = 0;  iTj(4,4,3) = 1;

%3T4
iTj(1,1,4) = -1; iTj(1,2,4) = 0; iTj(1,3,4) = 0;  iTj(1,4,4) = 0.1455;
iTj(2,1,4) = 0; iTj(2,2,4) = 0; iTj(2,3,4) = 1;  iTj(2,4,4) = 0;
iTj(3,1,4) = 0; iTj(3,2,4) = 1; iTj(3,3,4) = 0;  iTj(3,4,4) = 0.3265;
iTj(4,1,4) = 0; iTj(4,2,4) = 0; iTj(4,3,4) = 0;  iTj(4,4,4) = 1;

%4T5
iTj(1,1,5) = -1; iTj(1,2,5) = 0; iTj(1,3,5) = 0;  iTj(1,4,5) = 0;
iTj(2,1,5) = 0; iTj(2,2,5) = 0; iTj(2,3,5) = -1;  iTj(2,4,5) = -0.035;
iTj(3,1,5) = 0; iTj(3,2,5) = -1; iTj(3,3,5) = 0;  iTj(3,4,5) = 0;
iTj(4,1,5) = 0; iTj(4,2,5) = 0; iTj(4,3,5) = 0;  iTj(4,4,5) = 1;

%5T6
iTj(1,1,6) = 1; iTj(1,2,6) = 0; iTj(1,3,6) = 0;  iTj(1,4,6) = 0;
iTj(2,1,6) = 0; iTj(2,2,6) = 0; iTj(2,3,6) = -1;  iTj(2,4,6) = 0;
iTj(3,1,6) = 0; iTj(3,2,6) = 1; iTj(3,3,6) = 0;  iTj(3,4,6) = 0.385;
iTj(4,1,6) = 0; iTj(4,2,6) = 0; iTj(4,3,6) = 0;  iTj(4,4,6) = 1;

%6T7
iTj(1,1,7) = 1; iTj(1,2,7) = 0; iTj(1,3,7) = 0;  iTj(1,4,7) = 0;
iTj(2,1,7) = 0; iTj(2,2,7) = 0; iTj(2,3,7) = 1;  iTj(2,4,7) = 0.153;
iTj(3,1,7) = 0; iTj(3,2,7) = -1; iTj(3,3,7) = 0;  iTj(3,4,7) = 0;
iTj(4,1,7) = 0; iTj(4,2,7) = 0; iTj(4,3,7) = 0;  iTj(4,4,7) = 1;
end
