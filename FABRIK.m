function [joint_positions, iterations] = FABRIK(joint_positions, link_lengths, target, tol, max_iter)
% FABRIK for planar manipulators (all links rotate in XZ plane about Y axis)
% Input joint_positions: Nx3 initial guess
% Output: updated joint_positions in XZ plane and iterations to converge

N = size(joint_positions, 1);

% Project everything to XZ plane
target = [target(1), 0, target(3)];

for iter = 1:max_iter
    %% Step 1: Forward reaching
    joint_positions(end,:) = target;

    for i = N-1:-1:1
        r = joint_positions(i+1,:) - joint_positions(i,:);
        r(2) = 0; % Remove Y component
        r = r / (norm(r) + eps);
        joint_positions(i,:) = joint_positions(i+1,:) - link_lengths(i) * r;
    end

    %% Step 2: Backward reaching
    joint_positions(1,:) = [0, 0, 0];  % Base fixed

    for i = 2:N
        r = joint_positions(i,:) - joint_positions(i-1,:);
        r(2) = 0;  % Remove Y component
        r = r / (norm(r) + eps);
        joint_positions(i,:) = joint_positions(i-1,:) + link_lengths(i-1) * r;
    end

    %% Check convergence
    err = norm(joint_positions(end,:) - target);
    if err < tol
        % fprintf('Converged in %d iterations with error %.6f\n', iter, err);
        iterations = iter;
        return;
    end
end

% fprintf('Max iterations reached with error %.6f\n', err);
iterations = max_iter;
end
