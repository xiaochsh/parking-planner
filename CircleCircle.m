function [isok, l1, k1, l2, k2] = CircleCircle(start_pose, goal_pose, veh)
    
    Lmin = inf;
    X = [2; -2; veh.RMin]; X0 = [0; 0; 0];
    times = 1; MAX_TIMES = 100;
    while norm(X - X0) > 0.001 && times < MAX_TIMES
        X0 = X;
        t1 = X0(1) / X0(3); t2 = X0(2) / X0(3);
        P = [t1 - t2 + start_pose.Theta - goal_pose.Theta;
            start_pose.X - goal_pose.X + 2.0 * X0(3) * sin(0.5 * t1) * cos(start_pose.Theta + 0.5 * t1) + 2.0 * X0(3) * sin(0.5 * t2) * cos(start_pose.Theta + t1 - 0.5 * t2);
            start_pose.Y - goal_pose.Y + 2.0 * X0(3) * sin(0.5 * t1) * sin(start_pose.Theta + 0.5 * t1) + 2.0 * X0(3) * sin(0.5 * t2) * sin(start_pose.Theta + t1 - 0.5 * t2)];
        gradP = [1.0 / X0(3), -1.0 / X0(3), (X0(2) - X0(1)) / X0(3) ^ 2;
            cos(start_pose.Theta + t1) - 2.0 * sin(0.5 * t2) * sin(start_pose.Theta + t1 - 0.5 * t2), cos(start_pose.Theta + t1 - t2), ...
            2.0 * sin(0.5 * t1) * cos(start_pose.Theta + 0.5 * t1) + 2.0 * sin(0.5 * t2) * cos(start_pose.Theta + t1 - 0.5 * t2) - t1 * cos(start_pose.Theta + t1) - t2 * cos(start_pose.Theta + t1 - t2) + 2.0 * t1 * sin(0.5 * t2) * sin(start_pose.Theta + t1 - 0.5 * t2);
            sin(start_pose.Theta + t1) + 2.0 * sin(0.5 * t2) * cos(start_pose.Theta + t1 - 0.5 * t2), sin(start_pose.Theta + t1 - t2), ...
            2.0 * sin(0.5 * t1) * sin(start_pose.Theta + 0.5 * t1) + 2.0 * sin(0.5 * t2) * sin(start_pose.Theta + t1 - 0.5 * t2) - t1 * sin(start_pose.Theta + t1) - t2 * sin(start_pose.Theta + t1 - t2) - 2.0 * t1 * sin(0.5 * t2) * cos(start_pose.Theta + t1 - 0.5 * t2)];
        if isnan(rcond(gradP)) || rcond(gradP) < 1e-12
            times = MAX_TIMES;
            break;
        end
        X = X0 - gradP \ P;
        times = times + 1;
    end
    
    L = abs(X(1)) + abs(X(2));
    if times < MAX_TIMES && L < Lmin
        Lmin = L;
        l1 = X(1);
        k1 = 1.0 / X(3);
        l2 = X(2);
        k2 = -1.0 / X(3);
    end
    
    X = [2; 2; veh.RMin]; X0 = [0; 0; 0];
    times = 1; MAX_TIMES = 100;
    while norm(X - X0) > 0.001 && times < MAX_TIMES
        X0 = X;
        t1 = X0(1) / X0(3); t2 = X0(2) / X0(3);
        P = [-t1 + t2 + start_pose.Theta - goal_pose.Theta;
            start_pose.X - goal_pose.X + 2.0 * X0(3) * sin(0.5 * t1) * cos(start_pose.Theta - 0.5 * t1) + 2.0 * X0(3) * sin(0.5 * t2) * cos(start_pose.Theta - t1 + 0.5 * t2);
            start_pose.Y - goal_pose.Y + 2.0 * X0(3) * sin(0.5 * t1) * sin(start_pose.Theta - 0.5 * t1) + 2.0 * X0(3) * sin(0.5 * t2) * sin(start_pose.Theta - t1 + 0.5 * t2)];
        gradP = [-1.0 / X0(3), 1.0 / X0(3), (X0(1) - X0(2)) / X0(3) ^ 2;
            cos(start_pose.Theta - t1) + 2.0 * sin(0.5 * t2) * sin(start_pose.Theta - t1 + 0.5 * t2), cos(start_pose.Theta - t1 + t2), ...
            2.0 * sin(0.5 * t1) * cos(start_pose.Theta - 0.5 * t1) + 2.0 * sin(0.5 * t2) * cos(start_pose.Theta - t1 + 0.5 * t2) + t1 * cos(start_pose.Theta - t1) - t2 * cos(start_pose.Theta - t1 + t2) - 2.0 * t1 * sin(0.5 * t2) * sin(start_pose.Theta - t1 + 0.5 * t2);
            sin(start_pose.Theta - t1) - 2.0 * sin(0.5 * t2) * cos(start_pose.Theta - t1 + 0.5 * t2), sin(start_pose.Theta - t1 + t2), ...
            2.0 * sin(0.5 * t1) * sin(start_pose.Theta - 0.5 * t1) + 2.0 * sin(0.5 * t2) * sin(start_pose.Theta - t1 + 0.5 * t2) - t1 * sin(start_pose.Theta - t1) - t2 * sin(start_pose.Theta - t1 + t2) + 2.0 * t1 * sin(0.5 * t2) * cos(start_pose.Theta - t1 + 0.5 * t2)];
        if isnan(rcond(gradP)) || rcond(gradP) < 1e-12
            times = MAX_TIMES;
            break;
        end
        X = X0 - gradP \ P;
        times = times + 1;
    end
    
    L = abs(X(1)) + abs(X(2));
    if times < MAX_TIMES && L < Lmin
        Lmin = L;
        l1 = X(1);
        k1 = -1.0 / X(3);
        l2 = X(2);
        k2 = 1.0 / X(3);
    end
    
    if Lmin < inf && abs(k1) < 1.0 / veh.RMin
        isok = true;
    else
        isok = false;
        l1 = 0.0;
        k1 = 0.0;
        l2 = 0.0;
        k2 = 0.0;
    end
end
