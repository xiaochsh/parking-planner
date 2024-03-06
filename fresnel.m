function [C, S] = fresnel(s, c, n)
    
    C = 0.0; S = 0.0;
    for i = 0 : n
       C = C + ((-1) ^ i) * (s ^ (4 * i + 1)) * ((c / 2) ^ (2 * i)) / (4 * i + 1) / factorial(2 * i);
       S = S + ((-1) ^ i) * (s ^ (4 * i + 3)) * ((c / 2) ^ (2 * i + 1)) / (4 * i + 3) / factorial(2 * i + 1); 
    end
end

