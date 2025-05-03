function createMat(filename, numPoints)
% createMAT - Generate random XZ-plane points and save to a MAT file.
% Inputs:
%   filename   - name of the .mat file to save (e.g., 'targets.mat')
%   numPoints  - number of random target points to generate

if nargin < 1
    filename = 'targets.mat';
end
if nargin < 2
    numPoints = 100;
end

% Random radius [0, 0.9]
radii = 0.9 * rand(1, numPoints);

% Random angles [0, 360) degrees -> radians
angles_deg = 360 * rand(1, numPoints);
angles_rad = deg2rad(angles_deg);

% Compute (X, Y, Z)
x = radii .* cos(angles_rad);
y = zeros(1, numPoints);        % Planar in Y
z = radii .* sin(angles_rad);

% Combine into matrix: 3 x numPoints
points = [x; y; z];

% Save to MAT file
save(filename, 'points');

fprintf('Saved %d random XZ-plane points to %s\n', numPoints, filename);
end
