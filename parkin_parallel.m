clear; close all; clc

veh.LF = 3.55;
veh.LB = 0.9;
veh.W = 1.98;
veh.RMin = 5;
veh.Rfo = sqrt((veh.RMin + 0.5 * veh.W) ^ 2 + veh.LF ^ 2);
veh.Color = 'g';
veh.LineWidth = 0.01;
disc_dist = 0.1;
margin = 0.1;

MAX_TIMES = 1000;

bound_x = 6;
bound_y = 4;
bounds = [bound_x, bound_y];
xmax = 8;
xmin = -10;
ymax = 6;
ymin = -3;

lot.W = 2.9;
lot.L = 5.6;
axis equal;
hold on;grid on;
xlim([xmin xmax])
ylim([ymin ymax])
plot([xmin, xmax], [bound_y, bound_y], 'k', 'LineWidth', 2)
plot([xmin, -lot.L, -lot.L, 0, 0, xmax], [0, 0, -lot.W, -lot.W, 0, 0], 'k', 'LineWidth', 2)

start = WayPoint(-3.2, 1.6, -0.1);
plot(start.X, start.Y, 'ro')
goal = WayPoint(-veh.LF - 0.5 * (lot.L - veh.LF -veh.LB), -0.5 * lot.W, 0.0);
plot(goal.X, goal.Y, 'ro')

theta_fac = atan(0.5 * veh.W / veh.LF);
theta_rac = atan(0.5 * veh.W / veh.LB);

obst_right = Point(0, 0);
obst_left = Point(-lot.L, 0);
obsts = [obst_right, obst_left];
%% ==== once gear shift ===========
if lot.L > veh.LB + margin + sqrt((veh.Rfo + margin) ^ 2 - (veh.RMin - 0.5 * lot.W) ^ 2)
    [status, length1, kappa1, length2, length3, kappa3, length4] = OnceOrTwiceGearShift(start, goal, obsts, bounds, veh, margin);
    if status
        [x, y, theta] = WayPointsOnClothoid(start.X, start.Y, start.Theta, kappa1, 0, length1, floor(abs(length1) / disc_dist), false);
        VehicleAnimation(x, y, theta, veh);
        middle_pose1 = CalcStopPointByLength(start, kappa1, length1);
        plot(middle_pose1.X, middle_pose1.Y, 'ro');
        [x, y, theta] = WayPointsOnClothoid(middle_pose1.X, middle_pose1.Y, middle_pose1.Theta, 0.0, 0, length2, floor(abs(length2) / disc_dist), false);
        VehicleAnimation(x, y, theta, veh);
        middle_pose2 = CalcStopPointByLength(middle_pose1, 0.0, length2);
        plot(middle_pose2.X, middle_pose2.Y, 'ro');
        [x, y, theta] = WayPointsOnClothoid(middle_pose2.X, middle_pose2.Y, middle_pose2.Theta, kappa3, 0, length3, floor(abs(length3) / disc_dist), false);
        VehicleAnimation(x, y, theta, veh);
        if length4 > 0.0
            end_pose = CalcStopPointByLength(middle_pose2, kappa3, length3);
            plot(end_pose.X, end_pose.Y, 'ro'); 
            [x, y, theta] = WayPointsOnClothoid(end_pose.X, end_pose.Y, end_pose.Theta, 0, 0, length4, floor(abs(length4) / disc_dist), false);
            [vehx, vehy] = VehicleAnimation(x, y, theta, veh);
            plot(vehx, vehy, 'k', 'LineWidth', 1.5)
        end
        return;
    end
    %% ======== more times gear shift (PSO) ============
    D = 3;
    K = 50;
    N = 40;
    LIMIT_X = [obst_right.X, 0.5 * veh.W + margin, 0.0; 
        obst_right.X + 0.5 * (veh.LF + veh.LB), 0.5 * veh.W + veh.RMin - veh.Rfo - margin + bound_y, 0.3];
    LIMIT_V = [-0.05, -0.05, -0.01; 0.05, 0.05, 0.01];
    c0 = 1.5;
    min_w = 1.2; max_w = 1.6;
    plot([LIMIT_X(1, 1), LIMIT_X(2, 1), LIMIT_X(2, 1), LIMIT_X(1, 1), LIMIT_X(1, 1)], ...
        [LIMIT_X(1, 2), LIMIT_X(1, 2), LIMIT_X(2, 2), LIMIT_X(2, 2), LIMIT_X(1, 2)], 'r--')

    x = zeros(N, D);
    v = zeros(N, D);
    p_best = zeros(N, D); % 个体粒子最优位置
    % g_best = zeros(1, D); % 群体例子最优位置
    g_best = [obst_right.X + 0.25 * (veh.LF + veh.LB), obst_right.Y + 1.5 * veh.W + margin, 0.0]; % TODO: calculate by vehicle parameters
    for n = 1 : N
        for d = 1 : D
            x(n, d) = LIMIT_X(1, d) + rand * (LIMIT_X(2, d) - LIMIT_X(1, d));
            v(n, d) = LIMIT_V(1, d) + rand * (LIMIT_V(2, d) - LIMIT_V(1, d));
        end
        p_best(n, :) = x(n, :);
    end

    gbest_fitness = KeyPoseFitness(start, goal, WayPoint(g_best(1), g_best(2), g_best(3)), obsts, bounds, veh, margin);
    for k = 1 : K
        c1 = 1.5 + sin(0.5 * pi * (1.0 - 2.0 * k / K));
        c2 = 1.5 + sin(0.5 * pi * (2.0 * k / K - 1.0));
        w = max_w - (max_w - min_w) * k / K;
        last_gbest_fitness = gbest_fitness;
        for n = 1 : N
            % 更新速度(核心公式)
            temp_v = w * v(n, :) + c1 * rand * (p_best(n, :) - x(n, :)) + c2 * rand * (g_best - x(n, :));
            % 速度限制
            v(n, :) = max(min(temp_v, LIMIT_V(2, :)), LIMIT_V(1, :));
            % 更新位置
            temp_x = x(n, :) + v(n, :);
            % 位置限制
            x(n, :) = max(min(temp_x, LIMIT_X(2, :)), LIMIT_X(1, :));
            % 更新p_best和g_best
            if KeyPoseFitness(start, goal, WayPoint(x(n, 1), x(n, 2), x(n, 3)), obsts, bounds, veh, margin) > ...
                    KeyPoseFitness(start, goal, WayPoint(p_best(n, 1), p_best(n, 2), p_best(n, 3)), obsts, bounds, veh, margin)
                p_best(n, :) = x(n, :);
            end
            pbest_fitness = KeyPoseFitness(start, goal, WayPoint(p_best(n, 1), p_best(n, 2), p_best(n, 3)), obsts, bounds, veh, margin);
            if pbest_fitness > gbest_fitness              
                g_best(1, :) = p_best(n, :);
                gbest_fitness = pbest_fitness;
            end
        end
        if abs(gbest_fitness - last_gbest_fitness) < 0.01 && gbest_fitness > 0
           break; 
        end
    end
    key_pose = WayPoint(g_best(1), g_best(2), g_best(3))
    % key_pose = WayPoint(-2.3, 0.7, 0.5); % TODO: calculate by vehicle parameters
    
    [cost, length1, kappa1, length2, kappa2, length3, kappa3, length4, length5, kappa5, length6] = ...
        KeyPoseFitness(start, goal, key_pose, obsts, bounds, veh, margin);
    paths = [length1, kappa1;
        length2, kappa2;
        length3, kappa3;
        length4, 0.0;
        length5, kappa5;
        length6, 0.0];
    if cost > 0.0
        start_pose = start;
        for i = 1 : size(paths, 1)
            plot(start_pose.X, start_pose.Y, 'ro');
            [x, y, theta] = WayPointsOnClothoid(start_pose.X, start_pose.Y, start_pose.Theta, paths(i, 2), 0, paths(i, 1), floor(abs(paths(i, 1)) / disc_dist), false);
            [vehx, vehy] = VehicleAnimation(x, y, theta, veh);
            start_pose = CalcStopPointByLength(start_pose, paths(i, 2), paths(i, 1));
        end
        plot(vehx, vehy, 'k', 'LineWidth', 1.5)
    end
else
    length1 = goal.X - obst_left.X - veh.LB - margin;
    target_length = length1;
    left = 0.0; right = length1;
    while abs(left - right) > 0.01
        if CheckCollision(WayPoint(goal.X - target_length, goal.Y, goal.Theta), obst_right, veh, margin)
            left = target_length;
        else
            right = target_length;
        end
        target_length = 0.5 * (left + right);
    end
    paths = [-target_length, 0.0];
    forward_start_pose = CalcStopPointByLength(goal, paths(1, 2), paths(1, 1));
    while CheckCollision(forward_start_pose, obst_right, veh, margin)
        [status, theta] = CalcForwardMaxThetaByObst(forward_start_pose, veh, obst_right, margin);
        if status
            paths(size(paths, 1) + 1, :) = [veh.RMin * theta, 1.0 / veh.RMin];
            backward_start_pose = CalcStopPointByLength(forward_start_pose, paths(end, 2), paths(end, 1));
        else
            break;
        end
        [status1, theta1] = CalcBackwardMaxThetaByLotLength(backward_start_pose, veh, lot, margin);
        [status2, theta2] = CalcBackwardMaxThetaByLotWidth(backward_start_pose, veh, lot, margin);
        if status1 && status2
            theta = min(theta1, theta2);
            paths(size(paths, 1) + 1, :) = [-veh.RMin * theta, -1.0 / veh.RMin];
            forward_start_pose = CalcStopPointByLength(backward_start_pose, paths(end, 2), paths(end, 1));
        else
            break;
        end
    end
    
    %% ======== more times gear shift (PSO) ============
    D = 4;
    K = 50;
    N = 40;
    LIMIT_X = [obst_right.X, 0.5 * veh.W + margin, 0.0, forward_start_pose.Theta; 
        obst_right.X + 0.5 * (veh.LF + veh.LB), 0.5 * veh.W + veh.RMin - veh.Rfo - margin + bound_y, 0.3, 0.9];
    LIMIT_V = [-0.05, -0.05, -0.01, -0.01; 0.05, 0.05, 0.01, 0.01];
    c0 = 1.5;
    min_w = 1.2; max_w = 1.6;
    plot([LIMIT_X(1, 1), LIMIT_X(2, 1), LIMIT_X(2, 1), LIMIT_X(1, 1), LIMIT_X(1, 1)], ...
        [LIMIT_X(1, 2), LIMIT_X(1, 2), LIMIT_X(2, 2), LIMIT_X(2, 2), LIMIT_X(1, 2)], 'r--')

    x = zeros(N, D);
    v = zeros(N, D);
    p_best = zeros(N, D); % 个体粒子最优位置
    % g_best = zeros(1, D); % 群体例子最优位置
    g_best = [obst_right.X + 0.25 * (veh.LF + veh.LB), obst_right.Y + 1.5 * veh.W + margin, 0.0, 0.8]; % TODO: calculate by vehicle parameters
    for n = 1 : N
        for d = 1 : D
            x(n, d) = LIMIT_X(1, d) + rand * (LIMIT_X(2, d) - LIMIT_X(1, d));
            v(n, d) = LIMIT_V(1, d) + rand * (LIMIT_V(2, d) - LIMIT_V(1, d));
        end
        p_best(n, :) = x(n, :);
    end

    gbest_fitness = InLotPoseFitness(start, forward_start_pose, g_best, obsts, bounds, veh, margin);
    for k = 1 : K
        c1 = 1.5 + sin(0.5 * pi * (1.0 - 2.0 * k / K));
        c2 = 1.5 + sin(0.5 * pi * (2.0 * k / K - 1.0));
        w = max_w - (max_w - min_w) * k / K;
        last_gbest_fitness = gbest_fitness;
        for n = 1 : N
            % 更新速度(核心公式)
            temp_v = w * v(n, :) + c1 * rand * (p_best(n, :) - x(n, :)) + c2 * rand * (g_best - x(n, :));
            % 速度限制
            v(n, :) = max(min(temp_v, LIMIT_V(2, :)), LIMIT_V(1, :));
            % 更新位置
            temp_x = x(n, :) + v(n, :);
            % 位置限制
            x(n, :) = max(min(temp_x, LIMIT_X(2, :)), LIMIT_X(1, :));
            % 更新p_best和g_best
            if InLotPoseFitness(start, forward_start_pose, x(n, :), obsts, bounds, veh, margin) > ...
                    InLotPoseFitness(start, forward_start_pose, p_best(n, :), obsts, bounds, veh, margin)
                p_best(n, :) = x(n, :);
            end
            pbest_fitness = InLotPoseFitness(start, forward_start_pose, p_best(n, :), obsts, bounds, veh, margin);
            if pbest_fitness > gbest_fitness              
                g_best(1, :) = p_best(n, :);
                gbest_fitness = pbest_fitness;
            end
        end
        if abs(gbest_fitness - last_gbest_fitness) < 0.01 && gbest_fitness > 0
           break; 
        end
    end
    
    [cost, length1, kappa1, length2, kappa2, length3, kappa3, length4, kappa4] = ...
        InLotPoseFitness(start, forward_start_pose, g_best, obsts, bounds, veh, margin);
    
    paths(size(paths, 1) + 1, :) = [veh.RMin * (g_best(1, 4) - forward_start_pose.Theta), 1.0 / veh.RMin];
    paths(size(paths, 1) + 1, :) = [-length4, kappa4];
    paths(size(paths, 1) + 1, :) = [-length3, kappa3];
    paths(size(paths, 1) + 1, :) = [-length2, kappa2];
    paths(size(paths, 1) + 1, :) = [-length1, kappa1];
    
    ori_paths = paths;
    for i = 1 : size(ori_paths, 1)
        paths(size(ori_paths, 1) - i + 1, 1) = -ori_paths(i, 1);
        paths(size(ori_paths, 1) - i + 1, 2) = ori_paths(i, 2);
    end 
    
    start_pose = start;
    for i = 1 : size(paths, 1)
        plot(start_pose.X, start_pose.Y, 'ro');
        [x, y, theta] = WayPointsOnClothoid(start_pose.X, start_pose.Y, start_pose.Theta, paths(i, 2), 0, paths(i, 1), floor(abs(paths(i, 1)) / disc_dist), false);
        [vehx, vehy] = VehicleAnimation(x, y, theta, veh);
        start_pose = CalcStopPointByLength(start_pose, paths(i, 2), paths(i, 1));
    end
    plot(vehx, vehy, 'k', 'LineWidth', 1.5)
end

function [cost, length1, kappa1, length2, kappa2, length3, kappa3, length4, kappa4] = ...
        InLotPoseFitness(start_pose, forward_start_pose, xn, obsts, bounds, veh, margin)
    obst_right = obsts(1); %obst_left = obsts(2);
    bound_y = bounds(2); bound_x = bounds(1); 
    
    length1 = 0.0; kappa1 = 0.0; 
    length2 = 0.0; kappa2 = 0.0; 
    length3 = 0.0; kappa3 = 0.0;
    length4 = 0.0; kappa4 = 0.0;
    
    goal_pose = CalcStopPointByLength(forward_start_pose, 1.0 / veh.RMin, (xn(1, 4) - forward_start_pose.Theta) * veh.RMin);
    key_pose = WayPoint(xn(1, 1), xn(1, 2), xn(1, 3));
    vfl_start = AxisLocalToGlobal(key_pose, WayPoint(veh.LF, 0.5 * veh.W, 0.0));
    vfr_start = AxisLocalToGlobal(key_pose, WayPoint(veh.LF, -0.5 * veh.W, 0.0));
    vrr_start = AxisLocalToGlobal(key_pose, WayPoint(-veh.LB, -0.5 * veh.W, 0.0));
    if bound_y - vfl_start.Y > margin
        cost = 0;
    else
        cost = -1;
        return;
    end
    if vrr_start.X > obst_right.X
        if vrr_start.Y - obst_right.Y > margin
%             cost = 0;
        else
            cost = -1;
            return;
        end
    else
        a = [obst_right.X - vrr_start.X, obst_right.Y - vrr_start.Y];
        b = [vfr_start.X - vrr_start.X, vfr_start.Y - vrr_start.Y];
        if (a(1) * b(2) - a(2) * b(1)) / (veh.LB + veh.LF) > margin
%             cost = 0;
        else
            cost = -1;
            return;
        end
    end
    
    [ls, ll1, ll2, lk2] = LineCircle(start_pose, key_pose, veh);
    if ls
        [cs, cl1, ck1, cl2, ck2] = CircleCircle(start_pose, key_pose, veh);
        if cs && abs(cl1) + abs(cl2) < abs(ll1) + abs(ll2)
            length1 = cl1;
            kappa1 = ck1;
            length2 = cl2;
            kappa2 = ck2;
        else
            length1 = ll1;
            kappa1 = 0.0;
            length2 = ll2;
            kappa2 = lk2;
        end
    else
        [cs, length1, kappa1, length2, kappa2] = CircleCircle(start_pose, key_pose, veh);
    end
    
    if ls || cs
        middle_pose3 = CalcStopPointByLength(start_pose, kappa1, length1);
        vfl_mp3 = AxisLocalToGlobal(middle_pose3, WayPoint(veh.LF, 0.5 * veh.W, 0));
        vfr_mp3 = AxisLocalToGlobal(middle_pose3, WayPoint(veh.LF, -0.5 * veh.W, 0));
        if bound_x - vfl_mp3.X < margin || bound_y - vfl_mp3.Y < margin || ...
                bound_x - vfr_mp3.X < margin || vfr_mp3.Y - obst_right.Y < margin
            cost = -1;
            return;
        end
    else
        cost = -1;
        return;
    end
    
    [status, length3, kappa3, length4] = CircleLine(key_pose, goal_pose, veh);

    if status
        center2 = CalcCenterPoint(key_pose, 1.0 / kappa3);
        Rfo = sqrt((1.0 / abs(kappa3) + 0.5 * veh.W) ^ 2 + veh.LF ^ 2);
        if CalcDistance(center2, obst_right) + margin < veh.RMin - 0.5 * veh.W && ...
                (center2.Y + Rfo + margin < bound_y || key_pose.Theta + atan(0.5 * veh.W / veh.LF) > 0.5 * pi)

        else
            cost = -1;
            return;
        end
    else
        cost = -1;
        return;
    end
    cost = cost + 10.0 / (abs(length1) + abs(length2) + abs(length3) + abs(length4));
end

function [status, theta] = CalcForwardMaxThetaByObst(start_pose, veh, obst, margin)
    theta = 0.5; theta0 = 0.0;
    times = 1; MAX_TIMES = 1000;
    while norm(theta - theta0) > 0.001 && times < MAX_TIMES
        theta0 = theta;
        end_pose_theta = start_pose.Theta + theta0;
        f = start_pose.X + 2.0 * veh.RMin * sin(0.5 * theta0) * cos(start_pose.Theta + 0.5 * theta0) + veh.LF * cos(end_pose_theta) + 0.5 * veh.W * sin(end_pose_theta) - (obst.X - margin);
        df = veh.RMin * cos(end_pose_theta) - veh.LB * sin(end_pose_theta) + 0.5 * veh.W * cos(end_pose_theta);
        if abs(df) < 1e-12
            times = MAX_TIMES;
            break;
        end
        theta = theta0 - f / df;
        times = times + 1;
    end
    status = times < MAX_TIMES;
end

function [status, theta] = CalcBackwardMaxThetaByLotLength(start_pose, veh, lot, margin)
    theta = 0.5; theta0 = 0.0;
    times = 1; MAX_TIMES = 1000;
    while norm(theta - theta0) > 0.001 && times < MAX_TIMES
        theta0 = theta;
        end_pose_theta = start_pose.Theta + theta0;
        f = start_pose.X - 2.0 * veh.RMin * sin(0.5 * theta0) * cos(start_pose.Theta + 0.5 * theta0) - veh.LB * cos(end_pose_theta) - 0.5 * veh.W * sin(end_pose_theta) - (-lot.L + margin);
        df = -veh.RMin * cos(end_pose_theta) + veh.LB * sin(end_pose_theta) - 0.5 * veh.W * cos(end_pose_theta);
        if abs(df) < 1e-12
            times = MAX_TIMES;
            break;
        end
        theta = theta0 - f / df;
        times = times + 1;
    end
    status = times < MAX_TIMES;
end

function [status, theta] = CalcBackwardMaxThetaByLotWidth(start_pose, veh, lot, margin)
    theta = 0.5; theta0 = 0.0;
    times = 1; MAX_TIMES = 1000;
    while norm(theta - theta0) > 0.001 && times < MAX_TIMES
        theta0 = theta;
        end_pose_theta = start_pose.Theta + theta0;
        f = start_pose.Y - 2.0 * veh.RMin * sin(0.5 * theta0) * sin(start_pose.Theta + 0.5 * theta0) - veh.LB * sin(end_pose_theta) - 0.5 * veh.W * cos(end_pose_theta) - (-lot.W + margin);
        df = -veh.RMin * sin(end_pose_theta) - veh.LB * cos(end_pose_theta) + 0.5 * veh.W * sin(end_pose_theta);
        if abs(df) < 1e-12
            times = MAX_TIMES;
            break;
        end
        theta = theta0 - f / df;
        times = times + 1;
    end
    status = times < MAX_TIMES;
end

function collision = CheckCollision(start_pose, obst_right, veh, margin)
   collision = (start_pose.X - veh.RMin * sin(start_pose.Theta) - obst_right.X) ^ 2 + ...
       (start_pose.Y + veh.RMin * cos(start_pose.Theta) - obst_right.Y) ^ 2 < (veh.Rfo + margin) ^ 2;
end

function [status, length1, kappa1, length2, length3, kappa3, length4] = OnceOrTwiceGearShift(start_pose, goal_pose, obsts, bounds, veh, margin)
    status = false;
    length1 = 0.0; kappa1 = 0.0; 
    length2 = 0.0; 
    length3 = 0.0; kappa3 = 0.0; 
    length4 = 0.0;
    obst_right = obsts(1); obst_left = obsts(2); bound_y = bounds(2);
    center1 = CalcCenterPoint(start_pose, -veh.RMin);
    if CalcDistance(center1, obst_right) + margin < veh.RMin - 0.5 * veh.W && ...
            (center1.Y + veh.Rfo + margin < bound_y || start_pose.Theta + atan(0.5 * veh.W / veh.LF) > 0.5 * pi)
        min_end_pose_x = obst_left.X + veh.LB + margin;
        max_end_pose_x = obst_right.X - veh.LF - margin;
        end_pose_x_offset_step = 0.1;
        back_step_count = ceil((goal_pose.X - min_end_pose_x) / end_pose_x_offset_step);
        for_step_count = floor((max_end_pose_x - goal_pose.X) / end_pose_x_offset_step);
        end_pose_x_offsets = [0.0, end_pose_x_offset_step * (1 : for_step_count), -end_pose_x_offset_step * (1 : back_step_count)];
        for i = 1 : (1 + for_step_count + back_step_count)
            end_pose = WayPoint(goal_pose.X + end_pose_x_offsets(i), goal_pose.Y, goal_pose.Theta);
            center2 = CalcCenterPoint(end_pose, veh.RMin);
            if CalcDistance(center2, obst_right) > veh.Rfo + margin
                [status, length1, kappa1, length2, length3, kappa3] = CircleLineCircle(start_pose, end_pose, veh);
                if status
                    middle_pose2 = CalcStopPointByLength(end_pose, kappa3, -length3);
                    vfr_mp2 = AxisLocalToGlobal(middle_pose2, WayPoint(veh.LF, -0.5 * veh.W, 0.0));
                    [line_start, line_end] = ExtendVectorByWayPoint(vfr_mp2);
                    if CalcPoint2LineDist(obst_right, line_start, line_end) > margin
                        length4 = goal_pose.X - end_pose.X;
                        break;
                    end
                end
            end
        end
    end
end

function [cost, length1, kappa1, length2, kappa2, length3, kappa3, length4, length5, kappa5, length6] = ...
        KeyPoseFitness(start_pose, goal_pose, key_pose, obsts, bounds, veh, margin)
    obst_right = obsts(1); obst_left = obsts(2);
    bound_y = bounds(2); bound_x = bounds(1); 
    
    length1 = 0.0; kappa1 = 0.0; 
    length2 = 0.0; kappa2 = 0.0; 
    length3 = 0.0; kappa3 = 0.0;
    length4 = 0.0;
    length5 = 0.0; kappa5 = 0.0;
    length6 = 0.0; 
    
    vfl_start = AxisLocalToGlobal(key_pose, WayPoint(veh.LF, 0.5 * veh.W, 0.0));
    vfr_start = AxisLocalToGlobal(key_pose, WayPoint(veh.LF, -0.5 * veh.W, 0.0));
    vrr_start = AxisLocalToGlobal(key_pose, WayPoint(-veh.LB, -0.5 * veh.W, 0.0));
    if bound_y - vfl_start.Y > margin
        cost = 0;
    else
        cost = -1;
        return;
    end
    if vrr_start.X > obst_right.X
        if vrr_start.Y - obst_right.Y > margin
%             cost = 0;
        else
            cost = -1;
            return;
        end
    else
        a = [obst_right.X - vrr_start.X, obst_right.Y - vrr_start.Y];
        b = [vfr_start.X - vrr_start.X, vfr_start.Y - vrr_start.Y];
        if (a(1) * b(2) - a(2) * b(1)) / (veh.LB + veh.LF) > margin
%             cost = 0;
        else
            cost = -1;
        end
    end
    
    center2 = CalcCenterPoint(key_pose, -veh.RMin);
    if CalcDistance(center2, obst_right) + margin < veh.RMin - 0.5 * veh.W && ...
            (center2.Y + veh.Rfo + margin < bound_y || key_pose.Theta + atan(0.5 * veh.W / veh.LF) > 0.5 * pi)
        
    else
        cost = -1;
        return;
    end
    
    bfound = false;
    min_end_pose_x = obst_left.X + veh.LB + margin;
    max_end_pose_x = obst_right.X - veh.LF - margin;
    end_pose_x_offset_step = 0.1;
    back_step_count = ceil((goal_pose.X - min_end_pose_x) / end_pose_x_offset_step);
    for_step_count = floor((max_end_pose_x - goal_pose.X) / end_pose_x_offset_step);
    end_pose_x_offsets = [0.0, end_pose_x_offset_step * (1 : for_step_count), -end_pose_x_offset_step * (1 : back_step_count)];
    
    for i = 1 : (1 + for_step_count + back_step_count)
        end_pose = WayPoint(goal_pose.X + end_pose_x_offsets(i), goal_pose.Y, goal_pose.Theta);
        center1 = CalcCenterPoint(end_pose, veh.RMin);
        if CalcDistance(center1, obst_right) > veh.Rfo + margin
            [status, length1, kappa1, length2, kappa2, length3, kappa3, length4, length5, kappa5] = ...
                    LineOrCircleCircleLineCircle(start_pose, end_pose, key_pose, veh);
            if status
                middle_pose1 = CalcStopPointByLength(end_pose, kappa5, -length5);
                vfr_mp1 = AxisLocalToGlobal(middle_pose1, WayPoint(veh.LF, -0.5 * veh.W, 0.0));
                [line_start, line_end] = ExtendVectorByWayPoint(vfr_mp1);
                if CalcPoint2LineDist(obst_right, line_start, line_end) > margin
                    length6 = goal_pose.X - end_pose.X;
                    bfound = true;
                    break;
                end
            end
        end
    end

    if bfound
        middle_pose3 = CalcStopPointByLength(start_pose, kappa1, length1);
        vfl_mp3 = AxisLocalToGlobal(middle_pose3, WayPoint(veh.LF, 0.5 * veh.W, 0));
        vfr_mp3 = AxisLocalToGlobal(middle_pose3, WayPoint(veh.LF, -0.5 * veh.W, 0));
        if bound_x - vfl_mp3.X < margin || bound_y - vfl_mp3.Y < margin || ...
                bound_x - vfr_mp3.X < margin || vfr_mp3.Y - obst_right.Y < margin
            cost = -1;
            return;
        end
    else
        cost = -1;
        return;
    end
    cost = cost + 10.0 / (abs(length1) + abs(length2) + abs(length3) + abs(length4) + abs(length5) + abs(length6));
end

function [status, length1, kappa1, length2, kappa2, length3, kappa3, length4, length5, kappa5] = ...
        LineOrCircleCircleLineCircle(start_pose, goal_pose, key_pose, veh)
    
    [ls, ll1, ll2, lk2] = LineCircle(start_pose, key_pose, veh);
    if ls
        [cs, cl1, ck1, cl2, ck2] = CircleCircle(start_pose, key_pose, veh);
        if cs && abs(cl1) + abs(cl2) < abs(ll1) + abs(ll2)
            length1 = cl1;
            kappa1 = ck1;
            length2 = cl2;
            kappa2 = ck2;
        else
            length1 = ll1;
            kappa1 = 0.0;
            length2 = ll2;
            kappa2 = lk2;
        end
    else
        [cs, length1, kappa1, length2, kappa2] = CircleCircle(start_pose, key_pose, veh);
    end
    
    if ls || cs
        [status, length3, kappa3, length4, length5, kappa5] = CircleLineCircle(key_pose, goal_pose, veh);
    else
        status = false;
        length3 = 0;
        kappa3 = 0;
        length4 = 0;
        length5 = 0;
        kappa5 = 0;
    end
end

function [status, length1, kappa1, length2, length3, kappa3] = CircleLineCircle(start_pose, goal_pose, veh)
    x = (start_pose.X - goal_pose.X) / veh.RMin;
    y = (start_pose.Y - goal_pose.Y) / veh.RMin;
    phi = mod2pi(start_pose.Theta - goal_pose.Theta);
    [isok, t, u, v] = LpSpRp(x, y, phi);
    if isok
        kappa1 = -1.0 / veh.RMin;
        length1 = v / kappa1;
        length2 = -u * veh.RMin;
        kappa3 = 1.0 / veh.RMin;
        length3 = -t / kappa3;
        status = true;
    else
        kappa1 = 0.0;
        length1 = 0.0;
        length2 = 0.0;
        kappa3 = 0.0;
        length3 = 0.0;
        status = false;
    end
end

function v = mod2pi(x)
    v = rem(x, 2 * pi); % sign(v) same as sign(x)
    if v < -pi
        v = v + 2 * pi;
    elseif v > pi
        v = v - 2 * pi;
    end
end