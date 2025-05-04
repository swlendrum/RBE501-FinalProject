% Planar FABRIK Demo: Circle in XZ Plane (3D View, Delayed Animation)
clear; clc; close all;

% Link lengths
link_lengths = [0.3; 0.3; 0.3];
N_links = length(link_lengths);
N_points = N_links + 1;

% Initial straight-up configuration
joint_positions = zeros(N_points, 3);
for i = 2:N_points
    joint_positions(i,:) = joint_positions(i-1,:) + [0, 0, link_lengths(i-1)];
end

% Spiral path in XZ plane
% nPts = 1000;           % Number of points along the spiral
% max_radius = 0.89;      % Final radius
% center = [0, 0, 0];     % Spiral center
% nTurns = 10;
% theta = linspace(0, 2*pi*nTurns, nPts);
% r = linspace(0, max_radius, nPts);
% path = [r .* cos(theta) + center(1);
%           zeros(1, nPts);
%           r .* sin(theta) + center(3)];

path = load('waypoints100k.mat');
path = path.points;
nPts = size(path, 2);

% Precompute FABRIK solutions
tic;
fprintf('Solving FABRIK for %d targets...\n', nPts);
solutions = zeros(N_points, 3, nPts);
total_iter = 0;

distances = zeros(1, nPts);
iters = zeros(1, nPts);

for i = 1:nPts
    target = path(:, i)';

    % Record distance from current end-effector to target BEFORE solving
    dist = norm(joint_positions(end,:) - target);
    distances(i) = dist;

    [joint_positions, iter] = FABRIK(joint_positions, link_lengths, target, 1e-6, 5000);
    iters(i) = iter;

    total_iter = total_iter + iter;
    solutions(:,:,i) = joint_positions;
end

totalTime = toc;
fprintf('Total time: %.3f seconds\n', totalTime);

fprintf('Total iterations: %d\n', total_iter);
fprintf('Average iterations: %.6f\n', total_iter / nPts);

figure;
plot(iters, distances, 'b.');
ylabel('Distance to Target (m)');
xlabel('Iterations');
title('FABRIK: Distance to Target vs. Iteration Count');
grid on;


% Animation
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('FABRIK - Planar 3R Robot Tracing Circle in XZ Plane');
% title('FABRIK - Planar 3R Robot 1000 Random Points')
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
view(3);

plot3(path(1,:), path(2,:), path(3,:), 'r.', 'MarkerSize', 3);
h = plot3(nan, nan, nan, '-o', 'LineWidth', 2, 'MarkerSize', 6);
target_marker = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');


for i = 1:nPts
    joint_positions = solutions(:,:,i);
    target = path(:,i);
    set(h, 'XData', joint_positions(:,1), ...
           'YData', joint_positions(:,2), ...
           'ZData', joint_positions(:,3));
    set(target_marker, 'XData', target(1), ...
                       'YData', target(2), ...
                       'ZData', target(3));
    drawnow;
    % pause(0.01);
end
% 
% % Set up video writer
% video = VideoWriter('fabrik_circle_animation.mp4', 'MPEG-4');
% video.FrameRate = 30;
% open(video);
% 
% for i = 1:nPts
%     joint_positions = solutions(:,:,i);
%     target = path(:,i);
% 
%     set(h, 'XData', joint_positions(:,1), ...
%            'YData', joint_positions(:,2), ...
%            'ZData', joint_positions(:,3));
%     set(target_marker, 'XData', target(1), ...
%                        'YData', target(2), ...
%                        'ZData', target(3));
%     drawnow;
% 
%     frame = getframe(gcf);
%     writeVideo(video, frame);
% end
% 
% close(video); % Finalize and save the video
% 
