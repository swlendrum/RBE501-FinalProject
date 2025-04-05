function R = axisangle2rot(omega,theta)
% AXISANGLE2ROT Convert axis-angle representation to rotation matrix
% 
%   R = axisangle2rot(omega, theta) returns the 3x3 rotation matrix R corresponding 
%   to the given axis of rotation omega and the angle of rotation theta, using the 
%   axis-angle representation. The function calculates the rotation matrix based on 
%   Rodrigues' rotation formula.
%   
%   The axis-angle representation is commonly used in 3D transformations, where omega 
%   is the unit vector representing the axis of rotation, and theta is the angle of rotation 
%   in radians. The rotation matrix R transforms vectors by rotating them around the 
%   axis omega by an angle of theta.
%
%   Input:
%       omega - A 3x1 vector representing the axis of rotation (unit vector).
%       theta - A scalar representing the angle of rotation (in radians).
%
%   Output:
%       R     - A 3x3 rotation matrix computed using the axis-angle formula:
%               R = I + sin(theta) * W + (1 - cos(theta)) * W^2
%               where I is the identity matrix and W is the skew-symmetric matrix 
%               of the vector omega.
%
%   Example:
%       omega = [1; 0; 0];   % Axis of rotation (e.g., along the x-axis)
%       theta = pi/4;         % Angle of rotation (45 degrees)
%       R = axisangle2rot(omega, theta);
%
%   See also: skew

    W = skew(omega);
    R = eye(3) + sin(theta)*W + (1-cos(theta))*(W^2);
    
end