function p = FABRIK(pTarget, jointDistances)
    % 3D FABRIK (Forward and Backward Reaching Inverse Kinematics) algorithm
    % pTarget: target position in 3D space
    % jointDistances: distances between joints
    % Returns the joint angles that achieve the target position
    % Initialize variables
    numJoints = length(jointDistances);
    p = zeros(numJoints, 3); % Initialize joint positions
    p(1, :) = [0, 0, 0]; % Start at the origin
    p(numJoints, :) = pTarget; % Set the end effector position to the target

    % Forward reaching phase
    for i = numJoints-1:-1:1
        % Calculate the direction from the current joint to the next joint
        direction = p(i+1, :) - p(i, :);
        % Normalize the direction vector
        direction = direction / norm(direction);
        % Update the position of the current joint
        p(i, :) = p(i+1, :) - direction * jointDistances(i);
    end
    % Backward reaching phase
    for i = 2:numJoints
        % Calculate the direction from the current joint to the previous joint
        direction = p(i-1, :) - p(i, :);
        % Normalize the direction vector
        direction = direction / norm(direction);
        % Update the position of the current joint
        p(i, :) = p(i-1, :) + direction * jointDistances(i-1);
    end
    % Calculate the final joint angles based on the positions
    jointAngles = zeros(numJoints, 1);
    for i = 1:numJoints-1
        % Calculate the angle between the current joint and the next joint
        direction = p(i+1, :) - p(i, :);
        % Normalize the direction vector
        direction = direction / norm(direction);
        % Calculate the angle using the dot product
        jointAngles(i) = acos(dot([1, 0, 0], direction));
    end
    % Return the joint angles
    p = jointAngles;
end

