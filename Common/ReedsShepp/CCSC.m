function [isok,path] = CCSC(x,y,phi,RSPathElemType)
    Lmin = single(inf);
    type = single(zeros(1,5));
    path = RSPath_generate(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSmLm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(5,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(5,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(6,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(6,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(9,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(9,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(10,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(10,:),-t,pi/2,-u,-v,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = LpRmSmLm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(7,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(7,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(8,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(8,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(11,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(11,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(12,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(12,:),-v,-u,pi/2,-t,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end