%% 3D FABRIK Demo

close all; clear; clc;

%% Initialize the model of the robot
link_lengths = [0.32; 0.225; 0.035; 0.225; 0.035; 0.055];
N_links = length(link_lengths);
N_points = N_links + 1;

%% Generate the task-space spiral path
nPts = 100;
fprintf('Generating task space path... ');
phi = linspace(0, 4*pi, nPts);
r = linspace(0, 0.3, nPts);
x = r .* cos(phi) + 0.4;
y = r .* sin(phi);
z = 0.2 * ones(1, nPts); % Flat spiral at constant height
path = [x; y; z];
fprintf('Done.\n');

%% Plot the spiral task-space path
figure;
plot3(path(1,:), path(2,:), path(3,:), 'r.-', 'MarkerSize', 10, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Spiral Task-Space Path');
view(3);

xlim([-0.2 1]);
ylim([-0.5 0.5]);
zlim([0 0.5]);

%% Define joint rotation axes
joint_axes = [
    0 0 1;  % Joint 1 rotates about Z
    0 1 0;  % Joint 2 rotates about Y
    0 1 0;  % Joint 3 rotates about Y
    0 0 1;  % Joint 4 rotates about Z
    0 1 0;  % Joint 5 rotates about Y
    0 0 1;  % Joint 6 rotates about Z
];

%% Solve IK for each target point
tic;
solutions = cell(1, nPts);
joint_angles = cell(1, nPts);

for ii = 1:nPts
    % Initial straight configuration along Z
    joint_positions = zeros(N_points,3);
    for k = 2:N_points
        joint_positions(k,:) = joint_positions(k-1,:) + [0 0 link_lengths(k-1)];
    end

    target_pos = path(:,ii)';
    
    joint_positions_new = FABRIK_R(joint_positions, link_lengths, target_pos, 1e-3, 5000);
    
    solutions{ii} = joint_positions_new;
    joint_angles{ii} = extract_joint_angles(joint_positions_new, joint_axes);
end
totalTime = toc;

fprintf('Solved FABRIK for %d target positions!\n', nPts);
fprintf('Total time: %.3f seconds\n', totalTime);

%% Animate Robot Following the Spiral Path
figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Robot Following Spiral Path (FABRIK with Moving Frames)');
view(3);

xlim([-1 1]);
ylim([-1 1]);
zlim([0 1.5]);

% Plot spiral path underneath
plot3(path(1,:), path(2,:), path(3,:), 'r--', 'LineWidth', 1.5);

h = plot3(0,0,0,'-o','LineWidth',2,'MarkerSize',6);

% Create video writer
v = VideoWriter('fabrik_spiral.mp4', 'MPEG-4');
v.Quality = 100;
v.FrameRate = 20;
open(v);

for ii = 1:nPts
    joint_positions = solutions{ii};
    
    set(h, 'XData', joint_positions(:,1), ...
           'YData', joint_positions(:,2), ...
           'ZData', joint_positions(:,3));
    
    drawnow;
    
    % Write frame to video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v); % Finalize video
fprintf('Video saved as fabrik_spiral.mp4\n');
