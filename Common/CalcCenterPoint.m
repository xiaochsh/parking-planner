function sCenterPoint = CalcCenterPoint(sStartPoint, Radius)
%CALCCENTERPOINT 计算圆弧的圆心位置
% 输入:
% sStartPoint - 圆弧起始点
% Radius - 圆弧半径，符号规定：CWF- CWR+ ACWF+ ACWR-
% 输出:
% sCenterPoint - 圆心位置

    sCenterPoint = InitPoint;
    sCenterPoint.X = sStartPoint.X + abs(Radius) * cos(sStartPoint.Theta + sign(Radius) * single(pi / 2));
    sCenterPoint.Y = sStartPoint.Y + abs(Radius) * sin(sStartPoint.Theta + sign(Radius) * single(pi / 2));
end

