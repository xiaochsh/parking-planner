clear; close all

alpha = -0.2;
r = 5;
s = 2.0 * r * alpha;
c = -1.0 / (r * s);

tt = 0 : -0.001 : s;

cc = zeros(1, size(tt, 2));
ss = zeros(1, size(tt, 2));

for idx = 1 : size(tt, 2)
    [C, S] = Fresnel(tt(idx), c, 3);
    cc(idx) = C;
    ss(idx) = S;
end

plot(cc, ss)

sum = 0.0;
for i = 2 : length(cc)
   sum = sum + sqrt((cc(i) - cc(i-1)) ^ 2 + (ss(i) - ss(i-1)) ^ 2);
end
sum