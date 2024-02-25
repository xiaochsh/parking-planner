function [isok,path] = CCCC(x,y,phi,RSPathElemType)
    Lmin = single(inf);
    type = single(zeros(1,5));
    path = RSPath_generate(type,0,0,0,0,0);
    [isok,t,u,v] = LpRupLumRm(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(3,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(3,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(4,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(4,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(3,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(3,:),-t,-u,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(4,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(4,:),-t,-u,-u,-v,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end