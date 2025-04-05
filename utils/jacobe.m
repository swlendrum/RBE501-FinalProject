function J = jacobe(S,q)
% JACOBE Compute the manipulator's body Jacobian matrix
% 
%   J = jacobe(S, q) computes the body Jacobian matrix of a robotic manipulator. This relates 
%   the joint velocities (q_dot) to the end-effector velocity (v) in the manipulator's 
%   task space. This function computes the Jacobian matrix using the screw-axis representation 
%   for each joint, iterating through the joint twists and applying the adjoint transformation 
%   in the body frame.
%
%   Input:
%       S     - A 6xN matrix where each column represents the twist coordinates for 
%               a joint of the manipulator, expressed in the body frame. Each twist is a 
%               6x1 vector consisting of the angular velocity (w) and linear velocity (v).
%       q     - An Nx1 vector of joint angles (or displacements), where N is the number 
%               of degrees of freedom of the manipulator. Each element q(i) represents 
%               the joint angle or displacement for the i-th joint.
%
%   Output:
%       J     - A 6xN body Jacobian matrix, where each column corresponds to the contribution 
%               of the i-th joint's twist to the end-effector velocity. The Jacobian maps 
%               joint velocities to the end-effector velocity in the body frame.
%
%   Example:
%       S = [S1, S2, S3];  % Twist coordinates for each joint in body frame
%       q = [theta1, theta2, theta3];  % Joint angles
%       J = jacobe(S, q);  % Compute Jacobian matrix in the body-fixed frame
%
%   See also: adjoint, twist2ht

    n = length(q);
    J = zeros(6, n);
    T = eye(4);
    
    for i = n:-1:1
        s = S(:, i);
        J(:, i) = adjoint(T) * s;
        T = T * twist2ht(s, -q(i));
    end
end
