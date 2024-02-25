function [bFindTarTheta, Theta] = CosKSinSolver(D, K, LearningRate, InitValue, MinValue, MaxValue, MaxIterTimes, Tol)
%COSKSINSOLVER 数值求解'cos(theta) + K * sin(theta) - D = 0'方程
%   Detailed explanation goes here

    bFindTarTheta = false;
    Theta0 = InitValue;
    for idx = 1 : MaxIterTimes
        F = cos(Theta0) + K * sin(Theta0) - D;
        DF = -sin(Theta0) + K * cos(Theta0);
        Theta = Theta0 - LearningRate * F / DF;
        
        if abs(Theta - Theta0) < Tol
            bFindTarTheta = true;
            break;
        else
            Theta0 = Theta;
        end
    end
    if Theta < MinValue || Theta > MaxValue
        bFindTarTheta = false;
    end

end

