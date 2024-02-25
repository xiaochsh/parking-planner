% formula 8.1
function [isok,t,u,v] = LpSpLp(x,y,phi)
    [t,u] = cart2pol(x-sin(phi),y-1+cos(phi));
    if t >= 0
        v = mod2pi(phi-t);
        if v >= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = single(0);
    u = single(0);
    v = single(0);
end




