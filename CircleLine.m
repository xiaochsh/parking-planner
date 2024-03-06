function [isok, l1, k1, l2] = CircleLine(start_pose, goal_pose, veh)
    p1 = [start_pose.X; start_pose.Y; start_pose.Theta];
    p2 = [goal_pose.X; goal_pose.Y; goal_pose.Theta];
    
    [x, y, th] = normalize(p1, p2);
    if abs(1.0 - cos(th)) > 1e-12 && abs(x * sin(th) - y * cos(th)) > 1e-12
        k1 = (1.0 - cos(th)) / (x * sin(th) - y * cos(th));
        l1 = th / k1;
        l2 = (x - sin(th) / k1) / cos(th);
        isok = abs(k1) < veh.RMin;
    else
        isok = false;
        l1 = 0.0;
        l2 = 0.0;
        k1 = 0.0;
    end
end