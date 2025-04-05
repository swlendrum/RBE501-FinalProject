%% RBE/ME 501 - Robot Dynamics - Spring 2025
%  Homework 3, Problem 3
%  Instructor: L. Fichera, <loris@wpi.edu>
close all; clear, clc

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

%% Calculate the IK
% Initialize a matrix to store the IK solutions
q = zeros(nPts,6);
within_bounds = 0;

for ii = 1 : nPts
    q0 = zeros(1,6);
    targetPose = V(:,ii);
    %q(ii,:) = ikin(S,M,q0,targetPose);
    FABRIK(targetPose(1:3), [0.32, 0.215, 0.035, 0.77, 0.035])
    q(ii,:) = FABRIK(targetPose(1:3), [0.32, 0.215, 0.035, 0.77, 0.035]);

    T = fkine(S, M, q(ii, :), 'space');
    pose = MatrixLog6(T);
    pose = debracket(pose);

    if norm(targetPose - pose) < 1e-6
        within_bounds = within_bounds + 1;
    end
end

disp("# Converged: " + within_bounds)