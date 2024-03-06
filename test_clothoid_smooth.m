clear; close all;

axis equal;
hold on;grid on;

paths = [1.99086700385127,-0.0619466618583951;
    2.14455106670527,0.0619466618583951;
    -3.32179939868716,-0.194021541425247;
    -1.58289588750442,0;
    -0.792128268246966,0.200000000000000;
    0.202409198879623,-0.200000000000000;
    -0.264952272044610,0.200000000000000;
    0.171462451498561,-0.200000000000000;
    -0.261678082058800,0.200000000000000;
    0.201282393568136,-0.200000000000000;
    -0.434635472780145,0.200000000000000;
    0.411603654771328,-0.200000000000000;
    -1.02995247771874,0.200000000000000;
    0.575000000000000,0];

veh.RMin = 5;

max_v = 2.0 / 3.6;
max_c = 1.0 / veh.RMin / max_v;

d = 0.5 * abs(paths(4, 1));

alpha11 = abs(paths(3, 1) * paths(3, 2));
r1 = 1.0 / abs(paths(3, 2));
[isok, alpha21] = Alpha(r1, d, alpha11);

alpha12 = abs(paths(5, 1) * paths(5, 2));
r2 = 1.0 / abs(paths(5, 2));
[isok, alpha22] = Alpha(r1, d, alpha12);

s1 = 2.0 * r1 * alpha21;
s2 = 2.0 * r2 * alpha22;

if s1 < abs(paths(3, 1)) && s2 < abs(paths(5, 1))
    l1 = paths(3, 1) + s1;
    k1 = paths(3, 2);
    c1 = 0.0;
    [xs, ys, ths, ks] = trajactory(0, 0, 0, k1, l1, 0.0, 0.1);
    plot(xs, ys)
    sss = ss;
    aks = ks;

    l2 = -s1;
    k2 = 0.0;
    c2 = 1.0 / (1.0 / paths(3, 2) * s1);
    [xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l2, c2, 0.1);
    plot(xs, ys)
    sss = [sss, ss + sss(end)];
    aks = [aks, ks];
    
    l3 = -s2;
    k3 = 0.0;
    c3 = 1.0 / (-1.0 / paths(5, 2) * s2);
    [xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l3, c3, 0.1);
    plot(xs, ys)
    sss = [sss, ss + sss(end)];
    aks = [aks, ks];

    l4 = paths(5, 1) + s2;
    k4 = paths(5, 2);
    c4 = 0.0;
    [xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l4, 0.0, 0.1);
    plot(xs, ys)
    sss = [sss, ss + sss(end)];
    aks = [aks, ks];

    figure(2)
    grid on;
    plot(sss, aks)
else
    
    s = 1.0 / veh.RMin / max_c;
    [xk, yk] = fresnel(s, max_c, 3);
    alpha2 = max_c / 2.0 * s ^ 2;
    n1 = xk - veh.RMin * sin(alpha2);
    n2 = yk - veh.RMin * (1.0 - cos(alpha2));
    alpha11 = abs(paths(3, 1) * paths(3, 2));
    n01 = n1 + n2 * tan(alpha11 / 2.0);
    alpha12 = abs(paths(5, 1) * paths(5, 2));
    n02 = n1 + n2 * tan(alpha12 / 2.0);
    
    if (n01 + n02) < abs(paths(4, 1))
        l1 = paths(3, 1) + s;
        k1 = paths(3, 2);
        c1 = 0.0;
        [ss, xs, ys, ths, ks] = trajactory(0, 0, 0, k1, l1, 0.0, 0.1);
        plot(xs, ys)
        sss = ss;
        aks = ks;

        l2 = -s;
        k2 = 0.0;
        c2 = 1.0 / (1.0 / paths(3, 2) * s);
        [ss, xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l2, c2, 0.1);
        plot(xs, ys)
        sss = [sss, ss + sss(end)];
        aks = [aks, ks];

        l3 = -s;
        k3 = 0.0;
        c3 = 1.0 / (-1.0 / paths(5, 2) * s);
        [ss, xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l3, c3, 0.1);
        plot(xs, ys)
        sss = [sss, ss + sss(end)];
        aks = [aks, ks];

        l4 = paths(5, 1) + s;
        k4 = paths(5, 2);
        c4 = 0.0;
        [ss, xs, ys, ths, ks] = trajactory(xs(end), ys(end), ths(end), ks(end), l4, 0.0, 0.1);
        plot(xs, ys)
        sss = [sss, ss + sss(end)];
        aks = [aks, ks];

        figure(2)
        grid on;
        plot(sss, aks)
        
    end
end

function [isok, alpha] = Alpha(r, d, theta)
   alpha = 0.2; alpha0 = 0.0;
   times = 1; MAX_TIMES = 100;
   while abs(alpha - alpha0) > 0.001
      alpha0 = mod2pi(alpha);
      s = 2.0 * r * alpha0;
      c = 1.0 / (r * s);
      [x, y] = fresnel(s, c, 3);
      f = x - d + (y - r) * tan(0.5 * theta) + r * tan(0.5 * theta) * cos(alpha0) - r * sin(alpha0);
      df = -r * tan(0.5 * theta) * sin(alpha0) - r * cos(alpha0);
      if abs(df) < 1e-12
          times = MAX_TIMES;
          break;
      end
      alpha = alpha0 - f / df;
      times = times + 1;
   end
   
   if times < MAX_TIMES
       isok = true;
   else
       isok = false;
   end 
end