function bAboveLine = AboveLine(sPoint, sLineStartPoint, sLineStopPoint)
%ABOVELINE 检测目标点是否在有向线段的左侧
% 输入:
% sPoint - 目标点
% sLineStartPoint - 有向线段起点
% sLineStopPoint - 有向线段终点
% 输出:
% bAboveLine - 是否在左侧
    
    tol = single(1e-3);
    Vec_SS = [sLineStopPoint.X - sLineStartPoint.X; sLineStopPoint.Y - sLineStartPoint.Y];
    Vec_SP = [sPoint.X - sLineStartPoint.X; sPoint.Y - sLineStartPoint.Y];
    Dir = Vec_SS(1) * Vec_SP(2) - Vec_SS(2) * Vec_SP(1);
    if Dir > tol
        bAboveLine = boolean(true);
    else
        bAboveLine = boolean(false);
    end
end

