function q = ikin(S, M, currentQ, targetPose)
    function pose = debracket(twist)
        pose = [twist(3,2) twist(1,3) twist(2,1) twist(1:3,4)']';
    end
    
    function J_a = jacoba(S,M,q,frame)
        J_s = jacob0(S, q);
    
        Jw = J_s(1:3, :);
        Jv = J_s(4:6, :);
        
        T_s = fkine(S,M,q,'space');
        p = T_s(1:3, 4);
        
        J_a = Jv - skew(p) * Jw;
    end

    function S = skew(w)
    
        S = [ 0    -w(3)   w(2);
              w(3)   0    -w(1);
             -w(2)   w(1)   0 ];
    end

    qlim = [-180.0 180.0;
            -125.0 125.0;
            -138.0 138.0;
            -270.0 270.0;
            -120.0 133.5;
            -270.0 270.0];

    qlim = qlim * (pi / 180);

    max_iter_a = 40;
    max_iter_0 = 5000;

    q = currentQ;

    currentPos = M(1:3, 4);
    targetPos = targetPose(4:6);


    for i = 1:max_iter_a

        % Calculate analytical jacobian
        J = jacoba(S, M, q, 'space');

        % Damped Least Squares
        lambda = 0.11;
        J_star = J' * pinv(J*J' + lambda^2 * eye);

        deltaQ = J_star * (targetPos - currentPos);

        % Apply joint update
        q = q + 0.1 * deltaQ';

        % Bound within joint limits
        q = max(min(q, qlim(:, 2)'), qlim(:, 1)');

        % Recalculate FK for new currentPos
        T = fkine(S,M,q,'space');
        currentPos = T(1:3, 4);

        if norm(targetPos - currentPos) < 1e-1
            % disp("A: " + i)
            break
        end
    end

    currentPose = MatrixLog6(T);
    currentPose = debracket(currentPose);
    
    for i = 1:max_iter_0

        J = jacob0(S, q);

        % Damped Least Squares ---
        lambda = 0.01;
        J_star = J' * pinv(J*J' + lambda^2 * eye);
        deltaQ = J_star * (targetPose - currentPose);
        % ------------------------

        % Update current joint values based on above calculation
        q = q + 0.1 * deltaQ';

        q = max(min(q, qlim(:, 2)'), qlim(:, 1)');

        T = fkine(S,M,q,'space');
        currentPose = MatrixLog6(T);
        currentPose = debracket(currentPose);

        if norm(targetPose - currentPose) < 1e-6
            % disp("0: " + i)
            break
        end
    end

    % q = mod(q + pi, 2*pi) - pi;
end