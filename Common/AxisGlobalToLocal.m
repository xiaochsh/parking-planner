function sTarPntInLoc = AxisGlobalToLocal(sOriPntInRef, sTarPntInRef)
%AXISGLOBALTOLOCAL 坐标变换：全局坐标系（参考系）->局部坐标系（浮动坐标系）
% 输入:
% sOriPntInRef - 局部坐标系原点在参考系中的坐标
% sTarPntInRef - 目标点在参考系中的坐标
% 输出:
% sTarPntInLoc - 目标点在局部坐标系中的坐标
    
    sTarPntInLoc = InitWayPoint;
    sTarPntInLoc.X = cos(sOriPntInRef.Theta) * (sTarPntInRef.X - sOriPntInRef.X) + sin(sOriPntInRef.Theta) * (sTarPntInRef.Y - sOriPntInRef.Y);
    sTarPntInLoc.Y = cos(sOriPntInRef.Theta) * (sTarPntInRef.Y - sOriPntInRef.Y) - sin(sOriPntInRef.Theta) * (sTarPntInRef.X - sOriPntInRef.X);
    sTarPntInLoc.Theta = sTarPntInRef.Theta - sOriPntInRef.Theta;
end

