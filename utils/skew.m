function S = skew(w)
% SKEW Compute the skew-symmetric matrix of a 3D vector
% 
%   S = skew(w) returns the 3x3 skew-symmetric matrix S corresponding to the 
%   3D vector w. The skew-symmetric matrix is commonly used in 3D robotics and 
%   spatial transformations to represent the cross-product operation.
%
%   Input:
%       w     - A 3x1 vector representing a 3D vector in space.
%
%   Output:
%       S     - A 3x3 skew-symmetric matrix that satisfies the cross-product 
%               operation for the vector w.
%
%   Example:
%       w = [1; 2; 3];   % A 3D vector
%       S = skew(w);     % Compute the skew-symmetric matrix

    S = [ 0    -w(3)   w(2);
          w(3)   0    -w(1);
         -w(2)   w(1)   0 ];
end