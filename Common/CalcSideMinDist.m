function SideMinDist = CalcSideMinDist(sTFirstPoint, sTSecondPoint, sBFirstPoint, sBSecondPoint)
%CALCSIDEMINDIST 此处显示有关此函数的摘要
%   此处显示详细说明
    if sTFirstPoint.Y < sBFirstPoint.Y
        TempDist1 = CalcPoint2LineDist(sTFirstPoint, sBFirstPoint, sBSecondPoint);
    else
        TempDist1 = CalcPoint2LineDist(sBFirstPoint, sTFirstPoint, sTSecondPoint);
    end
    if sTSecondPoint.Y > sBSecondPoint.Y
        TempDist2 = CalcPoint2LineDist(sTSecondPoint, sBFirstPoint, sBSecondPoint);
    else
        TempDist2 = CalcPoint2LineDist(sBSecondPoint, sTFirstPoint, sTSecondPoint);
    end
    SideMinDist = min(TempDist1, TempDist2);
end

