function joint_positions = fabrik_r(joint_positions, link_lengths, target, tol, max_iter)
    % FABRIK-R algorithm
    % Input:
    % - joint_positions: Nx3 array, initial joint positions
    % - link_lengths: (N-1)x1 array, length of each link
    % - target: 1x3 desired position of end-effector
    % - tol: tolerance to consider target reached
    % - max_iter: maximum number of iterations
    
    N = size(joint_positions, 1);
    
    for iter = 1:max_iter
        %% Step 1: Forward reaching
        joint_positions(end, :) = target; % Set end effector to target
    
        for i = N-1:-1:1
            % Constrain the joint based on previous (child) joint
            prev = i + 1;
            curr = i;
    
            dir = (joint_positions(curr,:) - joint_positions(prev,:));
            dir = dir / norm(dir + eps);
    
            % Move current joint to maintain link length
            joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(curr);
    
            % Apply 1-DOF constraint (plane constraint)
            joint_positions(curr,:) = project_to_plane(joint_positions(curr,:), joint_positions(prev,:), prev, joint_positions);
        end
    
        %% Step 2: Backward reaching
        joint_positions(1,:) = [0 0 0]; % Assume base is fixed at origin
    
        for i = 2:N
            prev = i - 1;
            curr = i;
    
            dir = (joint_positions(curr,:) - joint_positions(prev,:));
            dir = dir / norm(dir + eps);
    
            % Move current joint to maintain link length
            joint_positions(curr,:) = joint_positions(prev,:) + dir * link_lengths(prev);
    
            % Apply 1-DOF constraint
            joint_positions(curr,:) = project_to_plane(joint_positions(curr,:), joint_positions(prev,:), prev, joint_positions);
        end
    
        %% Step 3: Check convergence
        err = norm(joint_positions(end,:) - target);
        if err < tol
            fprintf('Converged in %d iterations with error %.6f\n', iter, err);
            return;
        end
    end
    
    fprintf('Reached max iterations (%d) with error %.6f\n', max_iter, err);
    
    end
    
    function p_new = project_to_plane(p_curr, p_prev, idx, joints)
    % Project p_curr onto the plane defined by previous joint constraints
    % For now: assume a fixed plane for each joint (simplified version)
    
    % --- In a full version, plane would be dynamically computed based on parent/child constraints
    % Here for simplicity, assume movement allowed in XY-plane (z = constant)
    plane_normal = [0 0 1];
    plane_point = p_prev;
    
    v = p_curr - plane_point;
    dist = dot(v, plane_normal);
    p_new = p_curr - dist * plane_normal;
    
    end