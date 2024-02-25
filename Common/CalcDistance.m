function Dist = CalcDistance(FirstPoint, SecondPoint)
%CALCDISTANCE 计算两点之间的距离
%   此处显示详细说明
    Dist = sqrt(max((FirstPoint.X - SecondPoint.X) ^ 2 + (FirstPoint.Y - SecondPoint.Y) ^ 2, single(0.00001)));
end

