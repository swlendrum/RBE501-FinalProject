function joint_angles = extract_joint_angles(joint_positions, joint_axes)
% Extract joint angles from solved joint positions
% - joint_positions: Nx3 array of points
% - joint_axes: 6x3 array (rotation axes at each joint)

N = size(joint_positions, 1);
joint_angles = zeros(N-1, 1); % one angle per joint

for i = 1:N-1
    p0 = joint_positions(i,:);
    p1 = joint_positions(i+1,:);

    dir = p1 - p0;
    dir = dir / (norm(dir) + eps);

    % Define reference direction (initial configuration)
    if all(joint_axes(i,:) == [0 0 1]) % z-axis rotation
        ref_dir = [0 1 0]; % along y when rotating about z
    elseif all(joint_axes(i,:) == [0 1 0]) % y-axis rotation
        ref_dir = [0 0 1]; % along z when rotating about y
    else
        ref_dir = [1 0 0]; % fallback
    end

    % Project dir onto plane normal to rotation axis
    n = joint_axes(i,:);
    dir_proj = dir - dot(dir, n) * n;
    dir_proj = dir_proj / (norm(dir_proj) + eps);

    ref_proj = ref_dir - dot(ref_dir, n) * n;
    ref_proj = ref_proj / (norm(ref_proj) + eps);

    % Angle between ref_proj and dir_proj
    cos_theta = dot(ref_proj, dir_proj);
    sin_theta = norm(cross(ref_proj, dir_proj));

    joint_angles(i) = atan2(sin_theta, cos_theta);
end
end
