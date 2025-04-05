function J = jacob0(S,q)
% JACOB0 Compute the manipulator's Jacobian matrix (body frame)
% 
%   J = jacob0(S, q) computes the space Jacobian matrix of a robotic manipulator. This relates 
%   the joint velocities (q_dot) to the end-effector velocity (v) in the manipulator's 
%   task space. This function computes the Jacobian matrix using the screw-axis representation 
%   for each joint, iterating through the joint twists and applying the adjoint transformation.
%
%   Input:
%       S     - A 6xN matrix where each column represents the twist coordinates for 
%               a joint of the manipulator. Each twist is a 6x1 vector consisting of 
%               the angular velocity (w) and linear velocity (v).
%       q     - An Nx1 vector of joint angles (or displacements), where N is the number 
%               of degrees of freedom of the manipulator. Each element q(i) represents 
%               the joint angle or displacement for the i-th joint.
%
%   Output:
%       J     - A 6xN Jacobian matrix, where each column corresponds to the contribution 
%               of the i-th joint's twist to the end-effector velocity. The Jacobian maps 
%               joint velocities to the end-effector velocity.
%
%   Example:
%       S = [S1, S2, S3];  % Twist coordinates for each joint
%       q = [theta1, theta2, theta3];  % Joint angles
%       J = jacob0(S, q);  % Compute Jacobian matrix in the body-fixed frame
%
%   See also: adjoint, twist2ht

    n = length(q);
    J = zeros(6, n);
    T = eye(4);
    
    for i = 1:n
        s = S(:, i);
        J(:, i) = adjoint(T) * s;
        T = T * twist2ht(s, q(i));
    end
end