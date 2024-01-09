function R = ComputeAngleAxis(theta,v)
%Implement here the Rodrigues formula
vx = zeros(3,3);

%Define the [vx] matrix values with v coordinates
vx(2,1) = v(3);
vx(3,1) = -v(2);
vx(1,2) = -v(3);
vx(3,2) = v(1);
vx(1,3) = v(2);
vx(2,3) = -v(1);

%identity matrix
I = eye(3);

%Rodriguez formula
R = I + sin(theta) * vx + (1 - cos(theta)) * vx^2;
end
