% function joint_positions = FABRIK_r(joint_positions, link_lengths, target, tol, max_iter)
% % FABRIK with moving frame plane constraints
% % Real robot version
% 
% N = size(joint_positions, 1);
% 
% % Local axes (same as screw axes)
% joint_axes = [
%     0 0 1;
%     0 1 0;
%     0 1 0;
%     0 0 1;
%     0 1 0;
%     0 0 1;
% ];
% 
% % Initialize rotation matrices
% R = repmat(eye(3), [1 1 N]);
% 
% for iter = 1:max_iter
%     %% Step 1: Forward reaching
%     joint_positions(end,:) = target;
% 
%     for i = N-1:-1:1
%         prev = i+1;
%         curr = i;
% 
%         dir = joint_positions(curr,:) - joint_positions(prev,:);
%         dir = dir / (norm(dir) + eps);
% 
%         joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(curr);
% 
%         % Plane normal updated dynamically
%         axis_local = joint_axes(curr,:)';
%         axis_world = R(:,:,prev) * axis_local;
% 
%         plane_normal = axis_world';
% 
%         % Project onto allowed plane
%         p0 = joint_positions(prev,:);
%         joint_positions(curr,:) = project_point_to_plane(joint_positions(curr,:), plane_normal, p0);
% 
%         % Update rotation matrix (approximate)
%         R(:,:,curr) = R(:,:,prev);
% 
%     end
% 
%     %% Step 2: Backward reaching
%     joint_positions(1,:) = [0 0 0];
% 
%     for i = 2:N
%         prev = i-1;
%         curr = i;
% 
%         dir = joint_positions(curr,:) - joint_positions(prev,:);
%         dir = dir / (norm(dir) + eps);
% 
%         joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(prev);
% 
%         % Plane normal updated dynamically
%         axis_local = joint_axes(prev,:)';
%         axis_world = R(:,:,prev) * axis_local;
% 
%         plane_normal = axis_world';
% 
%         % Project onto allowed plane
%         p0 = joint_positions(prev,:);
%         joint_positions(curr,:) = project_point_to_plane(joint_positions(curr,:), plane_normal, p0);
% 
%         % Update rotation matrix (approximate)
%         R(:,:,curr) = R(:,:,prev);
%     end
% 
%     %% Step 3: Convergence Check
%     err = norm(joint_positions(end,:) - target);
%     if err < tol
%         fprintf('Converged in %d iterations with error %.6f\n', iter, err);
%         return;
%     end
% end
% 
% fprintf('Reached max iterations (%d) with error %.6f\n', max_iter, err);
% 
% end
% 
% %% Helper Function
% function p_proj = project_point_to_plane(p, n, p0)
% % Project point p onto a plane with normal n through point p0
% n = n(:)'; % Ensure row vector
% v = p - p0;
% dist = dot(v, n);
% p_proj = p - dist * n;
% end


function joint_positions = FABRIK_r(joint_positions, link_lengths, target, tol, max_iter)
% FABRIK with joint axis constraints (moving frame)

N = size(joint_positions, 1);

% Joint rotation axes
joint_axes = [
    0 0 1;
    0 1 0;
    0 1 0;
    0 0 1;
    0 1 0;
    0 0 1;
];

% Initial local frame rotations (identity)
R = repmat(eye(3), [1 1 N]);

for iter = 1:max_iter
    %% Step 1: Forward reaching
    joint_positions(end,:) = target;

    for i = N-1:-1:1
        prev = i+1;
        curr = i;

        dir = joint_positions(curr,:) - joint_positions(prev,:);
        dir = dir / (norm(dir) + eps);

        joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(curr);

        % Axis in world frame
        axis_local = joint_axes(curr,:)';
        axis_world = R(:,:,prev) * axis_local;

        % Project current joint to axis-aligned plane
        normal = axis_world';
        p0 = joint_positions(prev,:);
        joint_positions(curr,:) = project_point_to_plane(joint_positions(curr,:), normal, p0);

        % Update rotation: align Z of current frame with current direction
        z_dir = (joint_positions(prev,:) - joint_positions(curr,:))';
        z_dir = z_dir / norm(z_dir + eps);
        up = axis_world; % Constrain direction = "up"

        x_dir = cross(up, z_dir); x_dir = x_dir / norm(x_dir + eps);
        y_dir = cross(z_dir, x_dir);
        R(:,:,curr) = [x_dir y_dir z_dir];
    end

    %% Step 2: Backward reaching
    joint_positions(1,:) = [0 0 0];

    for i = 2:N
        prev = i-1;
        curr = i;

        dir = joint_positions(curr,:) - joint_positions(prev,:);
        dir = dir / (norm(dir) + eps);

        joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(prev);

        % Axis in world frame
        axis_local = joint_axes(prev,:)';
        axis_world = R(:,:,prev) * axis_local;

        % Project current joint to axis-aligned plane
        normal = axis_world';
        p0 = joint_positions(prev,:);
        joint_positions(curr,:) = project_point_to_plane(joint_positions(curr,:), normal, p0);

        % Update rotation
        z_dir = (joint_positions(curr,:) - joint_positions(prev,:))';
        z_dir = z_dir / norm(z_dir + eps);
        up = axis_world;

        x_dir = cross(up, z_dir); x_dir = x_dir / norm(x_dir + eps);
        y_dir = cross(z_dir, x_dir);
        R(:,:,curr) = [x_dir y_dir z_dir];
    end

    %% Step 3: Check for convergence
    err = norm(joint_positions(end,:) - target);
    if err < tol
        return;
    end
end
end

%% Helper
function p_proj = project_point_to_plane(p, n, p0)
% Project point p onto a plane with normal n through point p0
n = n(:)';
v = p - p0;
dist = dot(v, n);
p_proj = p - dist * n;
end