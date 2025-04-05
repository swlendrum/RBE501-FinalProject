function AdT = adjoint(T)
% ADJOINT Compute the Adjoint of a transformation matrix
% 
%   AdT = adjoint(T) returns the adjoint matrix of the given transformation matrix T.
%   The transformation matrix T is assumed to be a 4x4 matrix where the upper-left
%   3x3 block represents a rotation matrix, and the last column represents the position vector.
%
%   The adjoint matrix AdT is a 6x6 matrix that is commonly used in robotics and 
%   spatial transformations, where:
%       - R is the rotation matrix (3x3),
%       - p is the position vector (3x1),
%       - skew(p) is the skew-symmetric matrix of the position vector p.
% 
%   Input:
%       T - A 4x4 transformation matrix.
%           T = [R, p;
%                0, 1]
%           where R is a 3x3 rotation matrix, and p is a 3x1 position vector.
%
%   Output:
%       AdT - A 6x6 adjoint matrix computed as:
%              AdT = [R, 0;
%                     skew(p) * R, R]
% 
%   Example:
%       T = [R, p;
%            0, 1];
%       AdT = adjoint(T);
%
%   See also: skew
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    AdT = [R zeros(3,3);
           skew(p)*R R];
end