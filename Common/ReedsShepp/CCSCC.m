function [isok,path] = CCSCC(x,y,phi,RSPathElemType)
    Lmin = single(inf);
    type = single(zeros(1,5));
    path = RSPath_generate(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSLmRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(17,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(17,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(18,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath_generate(RSPathElemType(18,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end