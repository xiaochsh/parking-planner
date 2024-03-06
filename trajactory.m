function [ss, xs, ys, ths, ks] = trajactory(x0, y0, th0, k0, s, c, ds)
   npts = floor(abs(s) / ds);
   ss = [(0 : npts) * sign(s) * ds, s];
   xs = zeros(1, npts + 2);
   ys = zeros(1, npts + 2);
   ths = zeros(1, npts + 2);
   ks = zeros(1, npts + 2);
   
   xs(1) = x0;
   ys(1) = y0;
   ths(1) = th0;
   ks(1) = k0;
   for idx = 2 : npts + 2
      ks(idx) = ks(idx - 1) + (ss(idx) - ss(idx - 1)) * c;
      ths(idx) = ths(idx - 1) + (ss(idx) - ss(idx - 1)) * ks(idx);
      xs(idx) = xs(idx - 1) + (ss(idx) - ss(idx - 1)) * cos(ths(idx));
      ys(idx) = ys(idx - 1) + (ss(idx) - ss(idx - 1)) * sin(ths(idx));
   end
end