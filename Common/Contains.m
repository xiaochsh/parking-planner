function bContains = Contains(sPolygon, sTargetPoint)
% CONTAINS 引射线法判断目标点是否在多边形内部，向上引垂直射线；若穿过的交点为奇数，则目标点在多边形内部；
% 输入:
% sPolygon - 四边形角点
% sTargetPoint - 目标点
% 输出:
% bContains - 是否在四边形内部

    u8CrossingTimes = uint8(0);
    for i = 1 : (length(sPolygon) - 1)
        Slope = (sPolygon(i + 1).Y - sPolygon(i).Y) / (sPolygon(i + 1).X - sPolygon(i).X);
        bCond1 = (sPolygon(i).X <= sTargetPoint.X) && (sPolygon(i + 1).X >= sTargetPoint.X);
        bCond2 = (sPolygon(i).X >= sTargetPoint.X) && (sPolygon(i + 1).X <= sTargetPoint.X);
        bAbove = sTargetPoint.Y <= Slope * (sTargetPoint.X - sPolygon(i).X) + sPolygon(i).Y;
        if ((bCond1 || bCond2) && bAbove)
            u8CrossingTimes = u8CrossingTimes + 1;
        else
            % empty
        end
    end
    bContains = mod(u8CrossingTimes, 2) ~= 0; 
end
