%%   Author: C. Mann <cpmann@wpi.edu>
%    Last modified: 05/03/2025
%    Generates data for Newton-Raphson, DLS, and GD IK methods
%    and visualizes model.

clear; clc; close all;

% 1. Spiral path
% max_radius = 0.9;     % Final radius
% center = [0, 0, 0];   % Center of spiral
% nTurns = 10;                           % Number of spiral turns
% theta = linspace(0, 2*pi*nTurns, nPts);
% r = linspace(0, max_radius, nPts);   % Radius increases linearly
% 
% % Parametrize spiral in XZ plane
% path = [r .* cos(theta) + center(1);   % X
%           zeros(1, nPts);                % Y = 0
%           r .* sin(theta) + center(3)];  % Z
% plot3(path(1,:), path(2,:), path(3,:), 'b-', 'LineWidth', 1.5);


% 2. Generate from .mat file
path = load('waypoints.mat');
path = path.points;
scatter3(path(1,:), path(2,:), path(3,:), 10, 'filled');

nPts = size(path, 2);
tol = 1e-6;

%% ROBOT
% Link lengths
l1 = 0.3; l2 = 0.3; l3 = 0.3;

% Screw axes (in space frame)
S1 = [0; 1; 0; 0; 0; 0];
S2 = [0; 1; 0; -l1; 0; 0];
S3 = [0; 1; 0; -(l1 + l2); 0; 0];
S = [S1 S2 S3];

% Home configuration of the end-effector
M = [eye(3), [0; 0; l1 + l2 + l3];
     0 0 0 1];

M01 = eye(4);
M12 = [eye(3), [0; 0; l1];
     0 0 0 1];
M23 = [eye(3), [0; 0; l1 + l2];
     0 0 0 1];

fprintf('Starting tests...\n')

%% Newton Raphson
tic;
% Calculate the inverse kinematics
currentPose = zeros(3, 1);
waypoints = zeros(3,nPts);

% Set the current joint variables
currentQ = zeros(1,3);

% Initialize a matrix to store the end-effector positions
positions_1 = zeros(3, nPts);
positions_2 = zeros(3, nPts);
positions_3 = zeros(3, nPts);
positions_ee = zeros(3, nPts);

% Keep track of total iterations
iterations = 1;

fprintf('N-R Progress: ');
nbytes = fprintf('0%%'); 

% Loop over all the points
for ii = 1:nPts
    % Generate the robot's pose
    targetPose = path(:,ii);

    % Inverse Kinematics
    while norm(targetPose - currentPose) > tol
        % Calculate the anlytic Jacobian from space twist
        J = jacoba(S, M, currentQ, 'space');

        % Newton Raphson
        deltaQ = pinv(J) * (targetPose - currentPose);

        % Update step
        currentQ = currentQ + deltaQ';

        % Update current pose using forward kinematics
        T = fkine(S, M, currentQ, 'space');
        currentPose = T(1:3, 4);

        iterations = iterations + 1;
    end
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nPts*100));

    % Store the positions for plotting
    T01 = M01;
    T12 = fkine(S(:,1), M12, currentQ(1), 'space');
    T23 = fkine(S(:,1:2), M23, currentQ(1:2), 'space');

    positions_1(:,ii) = T01(1:3, 4);
    positions_2(:,ii) = T12(1:3, 4);
    positions_3(:,ii) = T23(1:3, 4);
    positions_ee(:, ii) = currentPose;

    waypoints(:,ii) = currentQ';
end

fprintf('\nNewton Raphson: %d iterations, average %f iterations\n',iterations, iterations / nPts);
totalTime = toc;
fprintf('Total time: %.3f seconds\n', totalTime);

%% Damped Least Squares
tic;
% Calculate the inverse kinematics
currentPose = zeros(3, 1);
waypoints = zeros(3,nPts);

% Set the current joint variables
currentQ = zeros(1,3);

% Initialize a matrix to store the end-effector positions
positions_1 = zeros(3, nPts);
positions_2 = zeros(3, nPts);
positions_3 = zeros(3, nPts);
positions_ee = zeros(3, nPts);

% Keep track of total iterations
iterations = 1;

fprintf('DLS Progress: ');
nbytes = fprintf('0%%'); 

% Loop over all the points
for ii = 1:nPts
    % Generate the robot's pose
    targetPose = path(:,ii);

    % Inverse Kinematics
    while norm(targetPose - currentPose) > tol
        % Calculate the anlytic Jacobian from space twist
        J = jacoba(S, M, currentQ, 'space');

        % Damped Least Squares
        lambda = 0.45;
        Jmult = J.' / (J*J.' + lambda^2*eye(size(J,1)));
        deltaQ = Jmult * (targetPose - currentPose);

        % Update step
        currentQ = currentQ + deltaQ';

        % Update current pose using forward kinematics
        T = fkine(S, M, currentQ, 'space');
        currentPose = T(1:3, 4);

        iterations = iterations + 1;
    end
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nPts*100));

    % Store the positions for plotting
    T01 = M01;
    T12 = fkine(S(:,1), M12, currentQ(1), 'space');
    T23 = fkine(S(:,1:2), M23, currentQ(1:2), 'space');

    positions_1(:,ii) = T01(1:3, 4);
    positions_2(:,ii) = T12(1:3, 4);
    positions_3(:,ii) = T23(1:3, 4);
    positions_ee(:, ii) = currentPose;

    waypoints(:,ii) = currentQ';
end

fprintf('\nDamped Least Squares: %d iterations, average %f iterations\n',iterations, iterations / nPts);
totalTime = toc;
fprintf('Total time: %.3f seconds\n', totalTime);
%% Gradient Descent
tic;
% Calculate the inverse kinematics
currentPose = zeros(3, 1);
waypoints = zeros(3,nPts);

% Set the current joint variables
currentQ = zeros(1,3);

% Initialize a matrix to store the end-effector positions
positions_1 = zeros(3, nPts);
positions_2 = zeros(3, nPts);
positions_3 = zeros(3, nPts);
positions_ee = zeros(3, nPts);

% Keep track of total iterations
iterations = 1;

fprintf('GD Progress: ');
nbytes = fprintf('0%%'); 

% Loop over all the points
for ii = 1:nPts
    % Generate the robot's pose
    targetPose = path(:,ii);

    % Inverse Kinematics
    while norm(targetPose - currentPose) > tol
        % Calculate the anlytic Jacobian from space twist
        J = jacoba(S, M, currentQ, 'space');

        % Gradiant Descent
        nablaH = pinv(J)*(targetPose - currentPose);
        alpha = 0.1;
        deltaQ = alpha * nablaH;

        % Update step
        currentQ = currentQ + deltaQ';

        % Update current pose using forward kinematics
        T = fkine(S, M, currentQ, 'space');
        currentPose = T(1:3, 4);

        iterations = iterations + 1;
    end
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nPts*100));

    % Store the positions for plotting
    T01 = M01;
    T12 = fkine(S(:,1), M12, currentQ(1), 'space');
    T23 = fkine(S(:,1:2), M23, currentQ(1:2), 'space');

    positions_1(:,ii) = T01(1:3, 4);
    positions_2(:,ii) = T12(1:3, 4);
    positions_3(:,ii) = T23(1:3, 4);
    positions_ee(:, ii) = currentPose;

    waypoints(:,ii) = currentQ';
end

fprintf('\nGradient Descent: %d iterations, average %f iterations\n',iterations, iterations / nPts);
totalTime = toc;
fprintf('Total time: %.3f seconds\n', totalTime);

%% Show on Robotics Toolbox
robot = make_robot();
% Show results
robot.plot(waypoints', 'movie', 'RBE-501-2025-Project-Output.avi');

%% Joint Positions (stick model)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Joint Positions');
xlim([-1.2 1.2]);
ylim([-1.5 1.5]);
zlim([-1.5 1.5]);

% Markers for joints
h1 = plot3(NaN, NaN, NaN, 'MarkerSize', 8, 'DisplayName', 'Joint 1');
h2 = plot3(NaN, NaN, NaN, 'MarkerSize', 8, 'DisplayName', 'Joint 2');
h3 = plot3(NaN, NaN, NaN, 'MarkerSize', 8, 'DisplayName', 'Joint 3');
hee = plot3(NaN, NaN, NaN, 'MarkerSize', 8, 'DisplayName', 'End Effector');

% Line for links
h_links = plot3(NaN(1,4), NaN(1,4), NaN(1,4), 'k-', 'LineWidth', 2, 'DisplayName', 'Links');

view(3)

for k = 1:nPts
    % Get positions at current timestep
    p1 = positions_1(:, k);
    p2 = positions_2(:, k);
    p3 = positions_3(:, k);
    pee = positions_ee(:, k);

    % Update joint markers
    set(h1, 'XData', p1(1), 'YData', p1(2), 'ZData', p1(3));
    set(h2, 'XData', p2(1), 'YData', p2(2), 'ZData', p2(3));
    set(h3, 'XData', p3(1), 'YData', p3(2), 'ZData', p3(3));
    set(hee, 'XData', pee(1), 'YData', pee(2), 'ZData', pee(3));

    % Update link line
    x = [p1(1), p2(1), p3(1), pee(1)];
    y = [p1(2), p2(2), p3(2), pee(2)];
    z = [p1(3), p2(3), p3(3), pee(3)];
    set(h_links, 'XData', x, 'YData', y, 'ZData', z);

    drawnow;
    pause(0.01); % Adjust for desired animation speed
end