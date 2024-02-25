function sTarPntInRef = AxisLocalToGlobal(sOriPntInRef, sTarPntInLoc)
%AXISLOCALTOGLOBAL 坐标变换：局部坐标系（浮动坐标系）->全局坐标系（参考系）
% 输入:
% sOriPntInRef - 局部坐标系原点在参考系中的坐标
% sTarPntInLoc - 目标点在局部坐标系中的坐标
% 输出:
% sTarPntInRef - 目标点在全局坐标系中的坐标
    
    sTarPntInRef = InitWayPoint;
    sTarPntInRef.X = sOriPntInRef.X + cos(sOriPntInRef.Theta) * sTarPntInLoc.X - sin(sOriPntInRef.Theta) * sTarPntInLoc.Y;
    sTarPntInRef.Y = sOriPntInRef.Y + sin(sOriPntInRef.Theta) * sTarPntInLoc.X + cos(sOriPntInRef.Theta) * sTarPntInLoc.Y;
    sTarPntInRef.Theta = sTarPntInLoc.Theta + sOriPntInRef.Theta;
end

