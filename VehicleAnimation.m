function [vehx, vehy] = VehicleAnimation(x, y, theta, veh, varargin)
    
    if nargin > uint8(4)
        videoFWriter = varargin{1};
        record = true;
    else
        videoFWriter = nan;
        record = false;
    end
    plot(x, y, 'b')
    plot(x, y, '.')
    vehx = zeros(5, 1, 'single');
    vehy = zeros(5, 1, 'single');
    for i = 1 : length(theta)
        px = x(i);
        py = y(i);
        pth = theta(i);
        [vehx, vehy] = getVehTran([px, py, pth], veh);
        plot(vehx, vehy, 'Color', veh.Color, 'LineWidth', veh.LineWidth); % �����߿�
        pause(0.02)
        if record
            img = getframe(gcf);
            writeVideo(videoFWriter, img)
        end
    end
end

 % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
function [x, y] = getVehTran(vec, veh)

    CornerFLInChasis = [veh.LF; 0.5 * veh.W];
    CornerFRInChasis = [veh.LF; -0.5 * veh.W];
    CornerRLInChasis = [-veh.LB; 0.5 * veh.W];
    CornerRRInChasis = [-veh.LB; -0.5 * veh.W];
    R = [cos(vec(3)), -sin(vec(3)); sin(vec(3)), cos(vec(3))];
    CornerFLInLocal = [vec(1); vec(2)] + R * CornerFLInChasis;
    CornerFRInLocal = [vec(1); vec(2)] + R * CornerFRInChasis;
    CornerRLInLocal = [vec(1); vec(2)] + R * CornerRLInChasis;
    CornerRRInLocal = [vec(1); vec(2)] + R * CornerRRInChasis;
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [CornerFLInLocal(1),CornerFRInLocal(1),CornerRRInLocal(1),CornerRLInLocal(1),CornerFLInLocal(1)];
    y = [CornerFLInLocal(2),CornerFRInLocal(2),CornerRRInLocal(2),CornerRLInLocal(2),CornerFLInLocal(2)];
end