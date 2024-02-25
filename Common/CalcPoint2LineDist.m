function Dist = CalcPoint2LineDist(sPoint, sLineStartPoint, sLineEndPoint)
%CALCPOINT2LINEDIST 计算点到直线的距离
%   此处显示详细说明
    Dist = ((sPoint.X - sLineStartPoint.X) * (sLineEndPoint.Y - sLineStartPoint.Y) - ...
        (sPoint.Y - sLineStartPoint.Y) * (sLineEndPoint.X - sLineStartPoint.X)) / ...
        CalcDistance(sLineStartPoint, sLineEndPoint);
end

