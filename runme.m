% Planar FABRIK Demo: Circle in XZ Plane (3D View)
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
nPts = 600;           % Number of points along the spiral
max_radius = 0.9;     % Final radius
center = [0, 0, 0];   % Center of spiral

% Spiral parameters
nTurns = 10;                           % Number of spiral turns
theta = linspace(0, 2*pi*nTurns, nPts);
r = linspace(0, max_radius, nPts);   % Radius increases linearly

% Parametrize spiral in XZ plane
circle = [r .* cos(theta) + center(1);   % X
          zeros(1, nPts);                % Y = 0
          r .* sin(theta) + center(3)];  % Z

% Prepare animation
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('FABRIK - Planar 3R Robot Tracing Circle in XZ Plane (3D View)');
xlim([-1.2 1.2]);
ylim([-1.5 1.5]);
zlim([-1.5 1.5]);

% Initial plots
h = plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), '-o', ...
    'LineWidth', 2, 'MarkerSize', 6);
circle_plot = plot3(circle(1,:), circle(2,:), circle(3,:), 'r--');
target_marker = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

view(3); % 3D perspective view

% Loop through the circle
for i = 1:nPts
    target = circle(:,i)';
    joint_positions = FABRIK(joint_positions, link_lengths, target, 1e-4, 5000);
    
    % Update plot
    set(h, 'XData', joint_positions(:,1), ...
           'YData', joint_positions(:,2), ...
           'ZData', joint_positions(:,3));
       
    set(target_marker, 'XData', target(1), 'YData', target(2), 'ZData', target(3));
    
    drawnow;
    pause(0.01);
end
