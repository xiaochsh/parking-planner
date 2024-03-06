function [isok, l1, l2, k2] = LineCircle(start_pose, goal_pose, veh)
    p1 = [start_pose.X; start_pose.Y; start_pose.Theta];
    p2 = [goal_pose.X; goal_pose.Y; goal_pose.Theta];
    
    [x, y, th] = normalize(p1, p2);
    if abs(1.0 - cos(th)) > 1e-12 && abs(y) > 1e-12
        k2 = (1.0 - cos(th)) / y;
        l1 = x - sin(th) / k2;
        l2 = th / k2;
        isok = abs(k2) < veh.RMin;
    else
        l1 = 0.0;
        l2 = 0.0;
        k2 = 0.0;
        isok = false;
    end
end