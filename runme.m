%% RBE/ME 501 - Robot Dynamics - Spring 2025
%  Homework 3, Problem 3
%  Instructor: L. Fichera, <loris@wpi.edu>
close all; clear, clc

addpath('utils');

%% Initialize the model of the robot
% Screw Axes:
S = [0 0 1  0     0 0;
     0 1 0 -0.32  0 0;
     0 1 0 -0.545 0 0;
     0 0 1  0.035 0 0;
     0 1 0 -0.77  0 0;
     0 0 1  0.035 0 0]';

% Home configuration:
R = eye(3);
p = [0 0.035 0.835]';
M = [R p; 0 0 0 1];

%% Load the test configurations
load target_poses.mat
nPts = length(V);
numJoints = 3;

%% Calculate the IKs
% Initialize a matrix to store the IK solutions
q = zeros(nPts, numJoints);

nPts = 100;
fprintf('Generating task space path... ');
phi = linspace(0, 4*pi, nPts);
r = linspace(0, 0.3, nPts) ;
x = r .* cos(phi) + 0.4;
y = r  .* sin(phi);
z = 0.2 * ones(1,nPts);
path = [x; y; z];
fprintf('Done.\n');

% Get the home position of the robot
currentPose = M(1:3,4);

% Set the current joint variables/distances
currentQ = zeros(1,numJoints);
jointDistances = zeros(numJoints, 1);

% Loop over all the points
for ii = 1 : nPts
    % Generate the robot's pose
    targetPose = path(:, ii);

    % Initialize joint positions
    jointPositions = zeros(numJoints, 3);
    for j = 2:numJoints
        jointPositions(j,:) = jointPositions(j-1,:) + [jointDistances(j-1), 0, 0];
    end
    currentPose = jointPositions(end,:)';

    % Inverse Kinematics
    while norm(targetPose - currentPose) > 1e-3
        % Perform FABRIK
        jointPositions = FABRIK_r(jointPositions, targetPose, jointDistances);

        % % Original
        % jointPositions = ikin(S, M, jointPositions, targetPose);
        % 
        % Update current pose
        currentPose = jointPositions(end,:)';
    end
    q(ii,:) = jointPositions(:,1);
end