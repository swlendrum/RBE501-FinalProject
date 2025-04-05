function J_a = jacoba(S,M,q,frame)
% JACOBA Computes the analytical Jacobian of a serial manipulator.
%
%   J_a = jacoba(S, M, q, frame) computes the analytical Jacobian, which 
%   relates joint velocities to end-effector linear velocities while 
%   considering the manipulator's configuration.
%
%   Inputs:
%       S      - 6×n Screw axis matrix in the space frame
%       M      - 4×4 Home configuration (end-effector pose at zero position)
%       q      - n×1 Joint angles (configuration vector)
%       frame  - String specifying the frame ('space' or 'body')
%
%   Output:
%       J_a    - 3×n Analytical Jacobian (linear velocity mapping)
%
%   Description:
%   - If 'body' frame is selected, the function converts the screw axes to 
%     the space frame using space_screw(S, M).
%   - It computes the geometric Jacobian J using jacob0(S, q).
%   - Extracts the angular velocity Jacobian Jw and linear velocity Jacobian Jv.
%   - Computes the end-effector position p using forward kinematics.
%   - The analytical Jacobian J_a is derived as Jv - skew(p) * Jw.
%
%   See also: space_screw, jacob0, fkine, skew
%
%   Example Usage:
%       J_a = jacoba(S, M, q, 'space');

    if strcmp(frame, 'body')
        S = space_screw(S, M);
    end

    J = jacob0(S, q);

    Jw = J(1:3, :);
    Jv = J(4:6, :);
    
    T = fkine(S,M,q,'space');
    p = T(1:3, 4);
    
    J_a = Jv - skew(p) * Jw;
    
end