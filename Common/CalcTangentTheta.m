function Theta = CalcTangentTheta(StartPoint, EndPoint)
%CALCTANGENTTHETA 计算矢量正切角度
%   StartPoint - 起点
%   EndPoint - 终点
    Theta = atan2(EndPoint.Y - StartPoint.Y, EndPoint.X - StartPoint.X);
%     % 角度范围[0, 2*pi]
%     if Theta < single(0)
%         Theta = Theta + single(2 * pi);
%     end
end

