% Single Iteration of the FABRIK algorithm
function p = FABRIK(currPos, targetPos, jointDistances)
    % 3D FABRIK (Forward and Backward Reaching Inverse Kinematics) algorithm
    % currPos: Current position of the each joint (nx3 vector)
    % targetPos: Target position of the end effector (1x3 vector)
    % jointDistances: Distances between joints (nx1 vector)
    % Returns the joint angles that achieve the target position
    
    % Initialize variables
    numJoints = length(currPos);

    % Distance between root and target
    rootTargetDist = norm(currPos(1, :) - targetPos);

    % Check whether target is within reach
    if(rootTargetDist > sum(jointDistances))
        % The target is unreachable
        for i = 1:numJoints - 1
            % Find distance between target and joint
            r = norm(targetPos - currPos(i, :));

            lambda = jointDistances(i) / r;

            % New joint position update
            currPos(i + 1, :) = (1 - lambda) * currPos(i, :) + lambda * targetPos;
        end
   
   
    else
        % The target is reachable
        b = currPos(1, :);

        % Check distance between end effector and target
        % is greater than some tolerance
        diff = norm(currPos(numJoints, :) - targetPos);

        tol = 1e-3;
        while diff > tol
            % STAGE 1: FORWARD REACHING STAGE

            % Set end effector as target
            currPos(numJoints, :) = targetPos;

            for i = numJoints - 1:-1:1
                % Get distance r between new joint position and current joint
                r = norm(currPos(i + 1, :) - currPos(i, :));
                lambda = jointDistances(i) / r;
                
                % Find new joint position
                currPos(i, :) = (1 - lambda) * currPos(i + 1, :) + lambda * currPos(i, :);
            end


            % STAGE 2: BACKWARD REACHING STAGE

            % Set root as initial position (using temp variable)
            currPos(1, :) = b;

            for i = 1:numJoints - 1
                r = norm(currPos(i + 1, :) - currPos(i, :));
                lambda = jointDistances(i) / r;

                % Find new joint position
                currPos(i + 1, :) = (1 - lambda) * currPos(i, :) + lambda * currPos(i + 1, :);
            end

            % Update the distance to check
            diff = norm(currPos(numJoints, :) - targetPos);
        end
    end

    % Return the new positions
    p = currPos;
end