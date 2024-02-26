clear; close all; clc

veh.LF = 3.55;
veh.LB = 0.9;
veh.W = 1.98;
veh.RMin = 5;
veh.Color = 'c';
margin = 0.3;

MAX_TIMES = 1000;

bound_x = 6;
bound_y = 6;
bounds = [bound_x, bound_y];
xmax = 8;
xmin = -10;
ymax = 8;
ymin = -5;

lot.W = 2.8;
lot.L = 4.8;
axis equal;
hold on;grid on;
xlim([xmin xmax])
ylim([ymin ymax])
plot([xmin, xmax], [bound_y, bound_y], 'k', 'LineWidth', 2)
plot([xmin, -lot.W, -lot.W, 0, 0, xmax], [0, 0, -lot.L, -lot.L, 0, 0], 'k', 'LineWidth', 2)

start = WayPoint(-2.0, 1.7, -0.1);
plot(start.X, start.Y, 'ro')
goal = WayPoint(-0.5 * lot.W, -veh.LF, 0.5 * pi);
plot(goal.X, goal.Y, 'ro')

%% ==== once gear shift ===========
theta_fac = atan(0.5 * veh.W / veh.LF);
theta_rac = atan(0.5 * veh.W / veh.LB);

theta_max_flc = 0.5 * pi  - theta_fac;
if theta_max_flc > start.Theta
    r_min_flc = (bound_y - start.Y - margin - veh.LF / cos(theta_fac)) / (2.0 * sin(0.5 * (theta_max_flc - ...
        start.Theta)) * sin(0.5 * (theta_max_flc + start.Theta)));
else
    r_min_flc = veh.RMin;
end
% fprintf("r_min_flc: %.3f\n", r_min_flc)

theta_max_rlc = 0.5 * pi - theta_rac;
if theta_max_rlc > start.Theta
    r_max_rlc = (start.X + lot.W + margin + veh.LB / cos(theta_rac)) / (2.0 * sin(0.5 * (theta_max_rlc - ...
        start.Theta)) * cos(0.5 * (theta_max_flc + start.Theta)));
else
    r_max_rlc = 10000;
end
% fprintf("r_max_flc: %.3f\n", r_max_rlc)

obst_right = Point(0, 0);
obst_left = Point(-lot.W, 0);
obsts = [obst_right, obst_left];
r0 = 0;
r_min_dist = 100;
times = 1;
while abs(r_min_dist - r0) > 0.001 && times < MAX_TIMES
    r0 = r_min_dist;
    f = (start.X + r0 * sin(start.Theta) - obst_right.X) ^ 2 + (start.Y - r0 * cos(start.Theta) - obst_right.Y) ^ 2 - (r0 - 0.5 * veh.W - margin) ^ 2;
    df = 2.0 * sin(start.Theta) * (start.X + r0 * sin(start.Theta) - obst_right.Y) - 2.0 * cos(start.Theta) * (start.Y - r0 * cos(start.Theta) - obst_right.Y) - 2.0 * (r0 - 0.5 * veh.W - margin);
    r_min_dist = r0 - f / df;
    times = times + 1;
end
% fprintf("r_min_dist: %.3f\n", r_min_dist)

r_goal = (start.X - goal.X) / (1.0 - sin(start.Theta));
% fprintf("r_goal: %.3f\n", r_goal)
end_y = start.Y - r_goal * cos(start.Theta);

if r_goal > max(max(r_min_dist, r_min_flc), veh.RMin) && r_goal < r_max_rlc && end_y >= goal.Y && times < MAX_TIMES && r_min_dist > 0
    kappa = -1.0 / r_goal;
    length = (goal.Theta - start.Theta) / kappa;
    [x, y, theta] = WayPointsOnClothoid(start.X, start.Y, start.Theta, kappa, 0, length, uint16(abs(length) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    plot(goal.X, end_y, 'ro')
    [x, y, theta] = WayPointsOnClothoid(goal.X, end_y, goal.Theta, 0, 0, goal.Y - end_y, uint16((end_y - goal.Y) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    % fprintf("arc-line with arc-length (%.3f) kappa (%.3f) line-length (%.3f)\n", length, kappa, goal.Y - end_y)
    return;
else
    fprintf("once gear shift connection faild\n")
end

%% ======== twice gear shift ===========
[status, length1, kappa1, length2, kappa2, length3] = CircleCircleLine(start, goal, veh);
if status
    middle_pose = CalcStopPointByLength(start, kappa1, length1);
    end_pose = CalcStopPointByLength(middle_pose, kappa2, length2);
    center2 = Point(end_pose.X + veh.RMin, end_pose.Y);
    
    min_side_dist_pose = CalcStopPointByLength(end_pose, kappa2, veh.RMin * atan(abs((center2.Y - obst_right.Y) / (center2.X - obst_right.X))));
    min_side_dist = CalcDistance(center2, min_side_dist_pose) - CalcDistance(center2, obst_right) - 0.5 * veh.W;
    middle_flc_y = middle_pose.Y + veh.LF * sin(middle_pose.Theta + theta_fac) / cos(theta_fac);

    if bound_y - middle_flc_y > margin && min_side_dist > margin
        [x, y, theta] = WayPointsOnClothoid(start.X, start.Y, start.Theta, kappa1, 0, length1, round(abs(length1) / 0.1), false);
        VehicleAnimation(x, y, theta, veh);
        plot(middle_pose.X, middle_pose.Y, 'ro')
        [x, y, theta] = WayPointsOnClothoid(middle_pose.X, middle_pose.Y, middle_pose.Theta, kappa2, 0, length2, round(abs(length2) / 0.1), false);
        VehicleAnimation(x, y, theta, veh);
        plot(end_pose.X, end_pose.Y, 'ro')
        [x, y, theta] = WayPointsOnClothoid(end_pose.X, end_pose.Y, end_pose.Theta, 0, 0, length3, round(abs(length3) / 0.1), false);
        VehicleAnimation(x, y, theta, veh);
        return;
    else
        fprintf("twice gear shift collision faild\n")
    end
else
    fprintf("twice gear shift connection faild\n")
end

%% ======== more times gear shift (PSO) ============
D = 3;
K = 100;
N = 10;
LIMIT_X = [goal.X - min(0.5 * lot.W, 1.5), 0, 0.3; goal.X + min(0.5 * lot.W, 1.5), 3.0, 0.8];
LIMIT_V = [-0.05, -0.05, -0.01; 0.05, 0.05, 0.01];
c0 = 1.5;
min_w = 1.2; max_w = 1.6;
plot([LIMIT_X(1, 1), LIMIT_X(2, 1), LIMIT_X(2, 1), LIMIT_X(1, 1), LIMIT_X(1, 1)], ...
    [LIMIT_X(1, 2), LIMIT_X(1, 2), LIMIT_X(2, 2), LIMIT_X(2, 2), LIMIT_X(1, 2)], 'r--')

x = zeros(N, D);
v = zeros(N, D);
p_best = zeros(N, D); % 个体粒子最优位置
% g_best = zeros(1, D); % 群体例子最优位置
g_best = [goal.X - 0.5 * (goal.X - obst_left.X), 1.0, 0.5]; % TODO: calculate by vehicle parameters
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
[status, length1, kappa1, length2, kappa2, length3, kappa3, length4, kappa4, length5] = ...
                        LineOrCircleCircleCircleCircleLine(start, goal, key_pose, veh);
if status
    [x, y, theta] = WayPointsOnClothoid(start.X, start.Y, start.Theta, kappa1, 0, length1, uint16(abs(length1) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    middle_pose1 = CalcStopPointByLength(start, kappa1, length1);
    plot(middle_pose1.X, middle_pose1.Y, 'ro')
    [x, y, theta] = WayPointsOnClothoid(middle_pose1.X, middle_pose1.Y, middle_pose1.Theta, kappa2, 0, length2, uint16(abs(length2) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    plot(key_pose.X, key_pose.Y, 'ro')

    [x, y, theta] = WayPointsOnClothoid(key_pose.X, key_pose.Y, key_pose.Theta, kappa3, 0, length3, round(abs(length3) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    middle_pose2 = CalcStopPointByLength(key_pose, kappa3, length3);
    plot(middle_pose2.X, middle_pose2.Y, 'ro')
    [x, y, theta] = WayPointsOnClothoid(middle_pose2.X, middle_pose2.Y, middle_pose2.Theta, kappa4, 0, length4, round(abs(length4) / 0.1), false);
    VehicleAnimation(x, y, theta, veh);
    end_pose = CalcStopPointByLength(middle_pose2, kappa4, length4);
    plot(end_pose.X, end_pose.Y, 'ro')
    [x, y, theta] = WayPointsOnClothoid(end_pose.X, end_pose.Y, end_pose.Theta, 0, 0, length5, round(abs(length5) / 0.1), false);
    [vehx, vehy] = VehicleAnimation(x, y, theta, veh);
    plot(vehx, vehy, 'k', 'LineWidth', 1.5)
else
    fprintf("line-circle connection faild\n")
end

function cost = KeyPoseFitness(start_pose, goal_pose, key_pose, obsts, bounds, veh, margin)
    obst_right = obsts(1); obst_left = obsts(2);
    bound_x = bounds(1); bound_y = bounds(2);
    vrl_start = AxisLocalToGlobal(key_pose, WayPoint(-veh.LB, 0.5 * veh.W, key_pose.Theta));
    vrr_start = AxisLocalToGlobal(key_pose, WayPoint(-veh.LB, -0.5 * veh.W, key_pose.Theta));
    if vrl_start.X > obst_left.X && vrl_start.Y < obst_left.Y
        if vrl_start.X - obst_left.X > margin
            cost = 0;
        else
            cost = -1;
            return;
        end
    elseif vrr_start.X < obst_left.X && vrr_start.Y > obst_left.Y
        if vrr_start.Y - obst_left.Y > margin
            cost = 0;
        else
            cost = -1;
            return;
        end
    else
        a = [obst_left.X - vrl_start.X, obst_left.Y - vrl_start.Y];
        b = [vrr_start.X - vrl_start.X, vrr_start.Y - vrl_start.Y];
        if (a(1) * b(2) - a(2) * b(1)) / veh.W > margin
            cost = 0;
        else
            cost = -1;
        end
    end
    [status, length1, kappa1, length2, kappa2, length3, kappa3, length4, kappa4, length5] = ...
                        LineOrCircleCircleCircleCircleLine(start_pose, goal_pose, key_pose, veh);
    if ~status
        cost = -1;
        return;
    end
    middle_pose1 = CalcStopPointByLength(start_pose, kappa1, length1);
    middle_pose2 = CalcStopPointByLength(key_pose, kappa3, length3);
    end_pose = CalcStopPointByLength(middle_pose2, kappa4, length4);
    center3 = Point(end_pose.X + veh.RMin, end_pose.Y);
    
    vfl_mp1 = AxisLocalToGlobal(middle_pose1, WayPoint(veh.LF, 0.5 * veh.W, 0));
    vfr_mp1 = AxisLocalToGlobal(middle_pose1, WayPoint(veh.LF, -0.5 * veh.W, 0));
    if bound_x - vfl_mp1.X < margin || bound_y - vfl_mp1.Y < margin || ...
            bound_x - vfr_mp1.X < margin || vfr_mp1.Y - obst_right.Y < margin
        cost = -1;
        return;
    end

    vfl_mp2 = AxisLocalToGlobal(middle_pose2, WayPoint(veh.LF, 0.5 * veh.W, 0));
    vfr_mp2 = AxisLocalToGlobal(middle_pose2, WayPoint(veh.LF, -0.5 * veh.W, 0));
    if bound_x - vfl_mp2.X < margin || bound_y - vfl_mp2.Y < margin || bound_x - vfr_mp2.X < margin
        cost = -1;
        return;
    end
    
    min_side_dist_pose = CalcStopPointByLength(end_pose, kappa4, veh.RMin * atan(abs((center3.Y - obst_right.Y) / (center3.X - obst_right.X))));
    min_side_dist = CalcDistance(center3, min_side_dist_pose) - CalcDistance(center3, obst_right) - 0.5 * veh.W;
    if min_side_dist < margin || abs(kappa2) > 1.0 / veh.RMin 
        cost = -1;
        return;
    end
    cost = cost + 10.0 / (abs(length1) + abs(length2) + abs(length3) + abs(length4) + abs(length5));
end

function [status, length1, kappa1, length2, kappa2, length3, kappa3, length4, kappa4, length5] = ...
    LineOrCircleCircleCircleCircleLine(start_pose, goal_pose, key_pose, veh)
    if start_pose.Theta < -0.01
        [status1, length1, kappa1, length2, kappa2] = CircleCircle(start_pose, key_pose, veh);
    else
        [status1, length1, length2, kappa2] = LineCircle(start_pose, key_pose, veh);
        kappa1 = 0;
    end
    if status1
        [status2, length3, kappa3, length4, kappa4, length5] = CircleCircleLine(key_pose, goal_pose, veh);
        if status2
            status = true;
        else
            status = false;
        end
    else
        status = false;
        kappa3 = 0;
        length3 = 0;
        kappa4 = 0;
        length4 = 0;
        length5 = 0;
    end
end

function [status, length1, kappa1, length2, kappa2, length3] = CircleCircleLine(start_pose, goal_pose, veh)
    L = [2; 2]; L0 = [0; 0];
    times = 1; MAX_TIMES = 1000;
    while norm(L - L0) > 0.001 && times < MAX_TIMES
        L0 = L;
        theta1 = L0(1) / veh.RMin; theta2 = L0(2) / veh.RMin;
        P = [theta1 + theta2 + start_pose.Theta - goal_pose.Theta;
            2.0 * veh.RMin * (sin(0.5 * theta1) * cos(start_pose.Theta + 0.5 * theta1) - ...
            sin(0.5 * theta2) * cos(start_pose.Theta + theta1 + 0.5 * theta2)) + start_pose.X - goal_pose.X];
        gradP = [1.0 / veh.RMin, 1.0 / veh.RMin;
            cos(start_pose.Theta + theta1) + 2.0 * sin(theta2) * sin(start_pose.Theta + theta1 + 0.5 * theta2), ...
            -cos(start_pose.Theta + theta1 + theta2)];
        L = L0 - gradP \ P;
        if isnan(rcond(gradP)) || rcond(gradP) < 1e-12
            times = MAX_TIMES;
            break;
        end
        times = times + 1;
    end
    
    if times < MAX_TIMES
        kappa1 = 1.0 / veh.RMin;
        length1 = L(1);
        kappa2 = -1.0 / veh.RMin;
        length2 = -L(2);
        end_pose_y = start_pose.Y + 2.0 / kappa1 * sin(0.5 * kappa1 * length1) * sin(start_pose.Theta + 0.5 * kappa1 * length1) + ...
            2.0 / kappa2 * sin(0.5 * kappa2 * length2) * sin(start_pose.Theta + kappa1 * length1 + 0.5 * kappa2 * length2);
        length3 = goal_pose.Y - end_pose_y;
        status = length1 > 0 && length2 < 0 && length3 < 0;
    else
        status = false;
        kappa1 = 0;
        length1 = 0;
        kappa2 = 0;
        length2 = 0;
        length3 = 0;
    end
end

function [status, length1, kappa1, length2, kappa2] = CircleCircle(start_pose, goal_pose, veh)
    X = [2; -2; veh.RMin]; X0 = [0; 0; 0];
    times = 1; MAX_TIMES = 1000;
    while norm(X - X0) > 0.001 && times < MAX_TIMES
        X0 = X;
        theta1 = X0(1) / X0(3); theta2 = X0(2) / X0(3);
        P = [theta1 - theta2 + start_pose.Theta - goal_pose.Theta;
            start_pose.X - goal_pose.X + 2.0 * X0(3) * sin(0.5 * theta1) * cos(start_pose.Theta + 0.5 * theta1) + 2.0 * X0(3) * sin(0.5 * theta2) * cos(start_pose.Theta + theta1 - 0.5 * theta2);
            start_pose.Y - goal_pose.Y + 2.0 * X0(3) * sin(0.5 * theta1) * sin(start_pose.Theta + 0.5 * theta1) + 2.0 * X0(3) * sin(0.5 * theta2) * sin(start_pose.Theta + theta1 - 0.5 * theta2)];
        gradP = [1.0 / X0(3), -1.0 / X0(3), (X0(2) - X0(1)) / X0(3) ^ 2;
            cos(start_pose.Theta + theta1) - 2.0 * sin(0.5 * theta2) * sin(start_pose.Theta + theta1 - 0.5 * theta2), cos(start_pose.Theta + theta1 - theta2), ...
            2.0 * sin(0.5 * theta1) * cos(start_pose.Theta + 0.5 * theta1) + 2.0 * sin(0.5 * theta2) * cos(start_pose.Theta + theta1 - 0.5 * theta2) - theta1 * cos(start_pose.Theta + theta1) - theta2 * cos(start_pose.Theta + theta1 - theta2) + 2.0 * theta1 * sin(0.5 * theta2) * sin(start_pose.Theta + theta1 - 0.5 * theta2);
            sin(start_pose.Theta + theta1) + 2.0 * sin(0.5 * theta2) * cos(start_pose.Theta + theta1 - 0.5 * theta2), sin(start_pose.Theta + theta1 - theta2), ...
            2.0 * sin(0.5 * theta1) * sin(start_pose.Theta + 0.5 * theta1) + 2.0 * sin(0.5 * theta2) * sin(start_pose.Theta + theta1 - 0.5 * theta2) - theta1 * sin(start_pose.Theta + theta1) - theta2 * sin(start_pose.Theta + theta1 - theta2) - 2.0 * theta1 * sin(0.5 * theta2) * cos(start_pose.Theta + theta1 - 0.5 * theta2)];
        X = X0 - gradP \ P;
        if isnan(rcond(gradP)) || rcond(gradP) < 1e-12
            times = MAX_TIMES;
            break;
        end
        times = times + 1;
    end

    if times < MAX_TIMES
        length1 = X(1);
        kappa1 = 1.0 / X(3);
        length2 = X(2);
        kappa2 = -1.0 / X(3);

        status = length1 > 0 && length2 < 0 && kappa1 > 0;
    else
        status = false;
        kappa1 = 0;
        length1 = 0;
        kappa2 = 0;
        length2 = 0;
    end
end

function [status, length1, length2, kappa2] = LineCircle(start_pose, goal_pose, veh)
    X = [2; 2; veh.RMin]; X0 = [0; 0; 0];
    times = 1; MAX_TIMES = 1000;
    while norm(X - X0) > 0.001 && times < MAX_TIMES
        X0 = X;
        theta2 = X0(2) / X0(3);
        theta2_2 = 0.5 * theta2;
        P = [theta2 + start_pose.Theta - goal_pose.Theta;
            X0(1) * cos(start_pose.Theta) - 2.0 * X0(3) * sin(theta2_2) * cos(start_pose.Theta + theta2_2) + start_pose.X - goal_pose.X;
            X0(1) * sin(start_pose.Theta) - 2.0 * X0(3) * sin(theta2_2) * sin(start_pose.Theta + theta2_2) + start_pose.Y - goal_pose.Y];
        gradP = [0.0, 1.0 / X0(3), -X0(2) / X0(3) ^ 2;
            cos(start_pose.Theta), -cos(start_pose.Theta + theta2), -2.0 * sin(theta2_2) * cos(start_pose.Theta + theta2_2) + theta2 * cos(start_pose.Theta + theta2);
            sin(start_pose.Theta), -sin(start_pose.Theta + theta2), -2.0 * sin(theta2_2) * sin(start_pose.Theta + theta2_2) + theta2 * sin(start_pose.Theta + theta2)];
        if isnan(rcond(gradP)) || rcond(gradP) < 1e-12
            times = MAX_TIMES;
            break;
        end
        X = X0 - gradP \ P;
        times = times + 1;
    end
    if times < MAX_TIMES
        length1 = X(1);
        length2 = -X(2);
        kappa2 = -1.0 / X(3);
        status = length2 < 0 && kappa2 < 0;
    else
        status = false;
        length1 = 0;
        length2 = 0;
        kappa2 = 0;
    end
end