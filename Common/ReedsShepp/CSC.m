function [isok,calc_path] = CSC(x,y,phi,RSPathElemType)
    Lmin = single(inf);
    type = single(zeros(1,5));
    calc_path = RSPath_generate(type,0,0,0,0,0);
    [isok,t,u,v] = LpSpLp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(15,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(15,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(16,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,-y,phi); % timeflp + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(16,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(13,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(13,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(14,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            calc_path = RSPath_generate(RSPathElemType(14,:),-t,-u,-v,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end