function pose = debracket(twist)
% debracket - Extracts a 6D pose vector from a 4x4 twist matrix
% 
% Syntax:
%   pose = debracket(twist)
%
% Description:
%   This function extracts the relevant pose parameters from a given 4x4 
%   twist matrix representation. The function returns a 6x1 pose vector 
%   containing the angular velocity components (ω) followed by the 
%   translational velocity components (v).
%
% Input:
%   twist - A 4x4 matrix representing a twist in se(3). It has the form:
%           [  0  -w3   w2   v1 ;
%              w3   0  -w1   v2 ;
%             -w2   w1   0   v3 ;
%               0    0    0    0 ]
%
% Output:
%   pose - A 6x1 vector [w1; w2; w3; v1; v2; v3], where:
%          - [w1; w2; w3] represents the angular velocity components.
%          - [v1; v2; v3] represents the translational velocity components.
%
% Example:
%   twist = [  0  -3   2   4 ;
%              3   0  -1   5 ;
%             -2   1   0   6 ;
%              0   0   0   0 ];
%   pose = debracket(twist)
%   % Expected output: [1; 2; 3; 4; 5; 6]

    pose = [twist(3,2) twist(1,3) twist(2,1) twist(1:3,4)']';

end