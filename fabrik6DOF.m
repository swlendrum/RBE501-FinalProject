
function jointAngles = fabrik6DOF(targetPose, boneLengths, jointAxes)
    % Inputs:
    % - targetPose: [x; y; z; roll; pitch; yaw] (end-effector pose)
    % - boneLengths: link lengths between joints
    % - jointAxes: cell array of joint rotation axes (e.g. {'z', 'y', 'x'})

    % Outputs:
    % - jointAngles: joint angles in radians


    numJoints = length(boneLengths) + 1;
    maxIter = 100;
    epsilon = 1e-3;

    % Extract target position and orientation
    posTarget = targetPose(1:3);
    eul = targetPose(4:6);  % roll, pitch, yaw
    R_target = eul2rotm(eul');

    % Initialize joint positions along x-axis
    p = zeros(numJoints, 3);
    for i = 2:numJoints
        p(i, :) = p(i-1, :) + [boneLengths(i-1), 0, 0];
    end
    base = p(1, :);

    % FABRIK loop
    for iter = 1:maxIter
        % --- Forward ---
        p(end, :) = posTarget;
        for i = numJoints-1:-1:1
            r = norm(p(i+1,:) - p(i,:));
            l = boneLengths(i);
            p(i,:) = p(i+1,:) + (p(i,:) - p(i+1,:)) * (l / r);
        end

        % --- Backward ---
        p(1,:) = base;
        for i = 2:numJoints
            r = norm(p(i,:) - p(i-1,:));
            l = boneLengths(i-1);
            p(i,:) = p(i-1,:) + (p(i,:) - p(i-1,:)) * (l / r);
        end

        % Check position convergence
        if norm(p(end,:) - posTarget) < epsilon
            break;
        end
    end

    % --- Enforce end-effector orientation ---
    % Compute current direction of final link
    dir_curr = p(end,:) - p(end-1,:);
    dir_curr = dir_curr / norm(dir_curr);

    % Z-forward convention: define R_curr
    % Assume final link frame has Z along link direction
    z_axis = dir_curr;
    x_axis = cross([0,1,0], z_axis);  % arbitrary perpendicular
    if norm(x_axis) < 1e-3
        x_axis = cross([1,0,0], z_axis);
    end
    x_axis = x_axis / norm(x_axis);
    y_axis = cross(z_axis, x_axis);
    R_curr = [x_axis; y_axis; z_axis]';

    % Compute correction rotation
    R_correction = R_target * R_curr';

    % Optionally apply R_correction to virtual final frame — here for error checking
    % You could use this matrix to drive the last joint rotation (with axis constraints)

    % For now: extract angle between R_target and R_curr
    angle_err = acos((trace(R_correction) - 1) / 2);
    if angle_err > epsilon
        disp(['End-effector orientation error (deg): ', num2str(rad2deg(angle_err))]);
    end

    % --- Compute joint angles w.r.t. axes ---
    jointAngles = zeros(numJoints-1, 1);
    for i = 1:numJoints-1
        v = p(i+1,:) - p(i,:);
        v = v / norm(v);

        axis = jointAxes{i};
        switch axis
            case 'x'
                ref = [1, 0, 0];
            case 'y'
                ref = [0, 1, 0];
            case 'z'
                ref = [0, 0, 1];
            otherwise
                error('Invalid joint axis.');
        end

        jointAngles(i) = atan2(norm(cross(ref, v)), dot(ref, v));
    end
end
