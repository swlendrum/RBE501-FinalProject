function T = twist2ht(S,theta)
% TWIST2HT Convert twist representation to homogeneous transformation matrix
% 
%   T = twist2ht(S, theta) returns the homogeneous transformation matrix T 
%   corresponding to the given twist S and scalar angle theta.
%   The twist S is a 6-dimensional vector, where the first 3 elements represent 
%   the angular velocity (w) and the last 3 elements represent the linear velocity (v).
%   The function calculates the transformation matrix corresponding to the motion 
%   described by the twist using the Rodrigues' formula.
%   
%   The homogeneous transformation matrix T represents the rotation and translation 
%   as a 4x4 matrix that transforms points or vectors from one coordinate frame to another.
%
%   Input:
%       S     - A 6x1 twist vector, where:
%               S = [w; v]
%               w (3x1) is the angular velocity (or axis of rotation),
%               v (3x1) is the linear velocity (or linear displacement).
%       theta - A scalar representing the angle of rotation (in radians).
%
%   Output:
%       T     - A 4x4 homogeneous transformation matrix, which is constructed as:
%               T = [R, p;
%                    0, 1]
%               where R is the 3x3 rotation matrix, and p is the 3x1 translation vector.
%
%   Example:
%       S = [w; v];     % Twist (angular and linear velocities)
%       theta = pi/2;   % Rotation angle
%       T = twist2ht(S, theta);
%
%   See also: skew, axisangle2rot

    w = S(1:3);
    v = S(4:6);
    W = skew(w);
    
    R = axisangle2rot(w,theta);
    p = (eye(3)*theta + (1-cos(theta))*W + (theta-sin(theta))*(W^2)) * v;
    
    T = [  R   p;
         0 0 0 1];
end