function [x, y, th] = normalize(start_pose, goal_pose)
    pvec = goal_pose - start_pose;
    phi = mod2pi(start_pose(3));
    dcm = [cos(phi), -sin(phi); sin(phi), cos(phi)]; 
    tvec = dcm' * [pvec(1); pvec(2)];
    x = tvec(1); y = tvec(2); th = pvec(3);
end