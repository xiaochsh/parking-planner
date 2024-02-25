function sStopPoint = CalcStopPointByLength(sStartPoint, Kappa, Length)
%CALCSTOPPOINTBYLENGTH 计算圆弧终点
% 输入:
% sStartPoint - 起点, X Y Theta
% Kappa - 圆弧曲率，符号规定：CWF- CWR+ ACWF+ ACWR-
% Length - 圆弧长度，符号规定：CWF+ CWR- ACWF+ ACWR-
% 输出:
% sStopPoint - 终点, X Y Theta
sStopPoint = sStartPoint;

% 几何法+矢量法：
% 终点矢量 = 起点矢量 + 起点-终点矢量
% 起点-终点矢量计算：根据几何关系计算相角和模长（等腰三角形、勾股定义）
% 注意合理利用Kappa和Length的符号

if abs(Kappa) > single(1e-6)
    sStopPoint.Theta = sStartPoint.Theta + Length * Kappa; % 圆弧DKappa为0，0.5 * DKappa * Length ^ 2 = 0
    sStopPoint.X = sStartPoint.X + 2.0 / Kappa * sin(0.5 * Length * Kappa) * cos(sStartPoint.Theta + 0.5 * Length * Kappa);
    sStopPoint.Y = sStartPoint.Y + 2.0 / Kappa * sin(0.5 * Length * Kappa) * sin(sStartPoint.Theta + 0.5 * Length * Kappa);
else
    sStopPoint.Theta = sStartPoint.Theta; % 直线
    sStopPoint.X = sStartPoint.X + Length * cos(sStartPoint.Theta);
    sStopPoint.Y = sStartPoint.Y + Length * sin(sStartPoint.Theta);
end

