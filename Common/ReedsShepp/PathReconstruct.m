function [type,...
          pos_x,...
          pos_y,...
          pos_theta,...
          circle_x,...
          circle_y,...
          direction,...
          dl,...
          path_num]= PathReconstruct(path,...
                                     rmin,...
                                     RS_LEFT,...
                                     RS_STRAIGHT,...
                                     RS_RIGHT,...
                                     PP_FORWARD,...
                                     PP_REVERSE)
    type = path(1:5);
%     x = [];
%     y = [];
    seg = path(6:10);
    pvec = single([0,0,0]);
    RS_NOP = uint8(0);
    dl = single(zeros(1,5));
    dx = single(0);
    dy = single(0);
    pos_x = single(zeros(1,5));
    pos_y = single(zeros(1,5));
    pos_theta = single(zeros(1,5));
    circle_x = single(zeros(1,5));
    circle_y = single(zeros(1,5));
    direction = single(zeros(1,5));
    path_num = uint8(0);
    for i = 1:5        
        if type(i) == RS_STRAIGHT
            theta = pvec(3);
            dl(i) = rmin*seg(i);
            dvec = [dl(i)*cos(theta), dl(i)*sin(theta), 0];
            dx = pvec(1)+linspace(0,dvec(1));
            dy = pvec(2)+linspace(0,dvec(2));
%             x = [x,dx];
%             y = [y,dy];
            pvec = pvec+dvec;
            pos_x(i) = dx(end);
            pos_y(i) = dy(end);
            pos_theta(i) = theta;
            path_num = path_num + 1;
        elseif type(i) == RS_LEFT
            theta = pvec(3);
            dtheta = seg(i);
            cenx = pvec(1)-rmin*sin(theta);
            ceny = pvec(2)+rmin*cos(theta);
            circle_x(i) = cenx;
            circle_y(i) = ceny;
            t = theta-pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl(i) = dtheta*rmin;
            pos_x(i) = dx(end);
            pos_y(i) = dy(end);
            pos_theta(i) = theta;
            path_num = path_num + 1;
        elseif type(i) == RS_RIGHT
            theta = pvec(3);
            dtheta = -seg(i);
            cenx = pvec(1)+rmin*sin(theta);
            ceny = pvec(2)-rmin*cos(theta);
            circle_x(i) = cenx;
            circle_y(i) = ceny;
            t = theta+pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl(i) = -dtheta*rmin;
            pos_x(i) = dx(end);
            pos_y(i) = dy(end);
            pos_theta(i) = theta;
            path_num = path_num + 1;
        else
            % do nothing
        end
        if dl(i) > 0
%             plot(dx,dy,'b');
            direction(i) = uint8(PP_FORWARD);
        else
%             plot(dx,dy,'r');
            direction(i) = uint8(PP_REVERSE);
        end
    end
end