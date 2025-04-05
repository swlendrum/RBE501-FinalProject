function T = fkine(S,M,q,frame)
    % FKINE Forward kinematics using twist coordinates
    % 
    %   T = fkine(S, M, q, frame) computes the forward kinematics of a robotic manipulator 
    %   using the screw-axis representation. The function calculates the end-effector 
    %   transformation matrix T given the twist coordinates S, the home configuration 
    %   transformation matrix M, a vector of joint angles q, and the desired frame type. 
    %   It iteratively applies the screw motions (twists) for each joint and combines them 
    %   with the home configuration to compute the overall transformation matrix.
    %
    %   Input:
    %       S     - A 6xN matrix where each column represents the twist coordinates for 
    %               a joint of the manipulator. Each twist is a 6x1 vector consisting of 
    %               the angular velocity (w) and linear velocity (v).
    %       M     - A 4x4 homogeneous transformation matrix representing the home configuration 
    %               (i.e., the transformation from the base frame to the end-effector frame when 
    %               all joint angles are zero).
    %       q     - An Nx1 vector of joint angles (or displacements), where N is the number 
    %               of degrees of freedom of the manipulator. Each element q(i) represents 
    %               the joint angle or displacement for the i-th joint.
    %       frame - A string specifying whether to compute the transformation in the 'body' 
    %               or 'space' frame.
    %
    %   Output:
    %       T     - A 4x4 homogeneous transformation matrix that represents the final position 
    %               and orientation of the end-effector relative to the base frame, calculated 
    %               by applying the individual twists for each joint and incorporating the 
    %               home configuration.
    %
    %   Example:
    %       S = [S1, S2, S3];  % Twist coordinates for each joint
    %       M = eye(4);         % Home configuration (identity matrix for simplicity)
    %       q = [theta1, theta2, theta3];  % Joint angles
    %       T = fkine(S, M, q, 'body');  % Compute forward kinematics in body frame
    %
    %   See also: twist2ht
    
    T = eye(4);
    for i = 1:length(q)
        T = T * twist2ht(S(:,i),q(i));
    end
    
    if strcmp(frame, 'body')
        T = M * T;
    else
        T = T * M;
    end
end
