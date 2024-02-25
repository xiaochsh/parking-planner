function [sVecStartPoint, sVecEndPoint] = ExtendVectorByWayPoint(sWayPoint)
%EXTENDVECTORBYWAYPOINT WayPoint扩展为单位向量
% 输入:
% sWayPoint - 起点, X Y Theta
% 输出:
% sVecStartPoint - 向量起点, X Y
% sVecEndPoint - 向量终点, X Y

sVecStartPoint = Point(sWayPoint.X, sWayPoint.Y);
sVecEndPoint = Point(sWayPoint.X + cos(sWayPoint.Theta), sWayPoint.Y + sin(sWayPoint.Theta));
end

