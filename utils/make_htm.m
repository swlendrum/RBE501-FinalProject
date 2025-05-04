function T = make_htm(R, p)
% MAKEHTM Constructs a homogeneous transformation matrix from rotation and position.
% 
%   T = HTM(R, p) returns a 4x4 homogeneous transformation matrix T given:
%       R - a 3x3 rotation matrix
%       p - a 3x1 position vector (or 1x3, which will be transposed)

    % Ensure p is a column vector
    if isrow(p)
        p = p';
    end

    % Construct the homogeneous transformation matrix
    T = [R, p;
         0, 0, 0, 1];
end
