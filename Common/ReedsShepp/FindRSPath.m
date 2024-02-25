function path = FindRSPath(x,...
                           y,...
                           phi,...
                           rmin,...
                           RS_LEFT,...
                           RS_STRAIGHT,...
                           RS_RIGHT)


    x = x/rmin;
    y = y/rmin;
    
    RS_NOP = uint8(0);   

    RSPathElemType = [   
        RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP ;        %1
        RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP ;       %2
        RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP ;      %3
        RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP ;      %4
        RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP ;   %5
        RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP ;  %6
        RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP ;   %7
        RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP ;  %8
        RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP ;  %9
        RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP ;   %10
        RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP ;  %11
        RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP ;   %12
        RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP ;    %13
        RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP ;    %14
        RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP ;     %15
        RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP ;   %16
        RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT ; %17
        RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT   %18
        ];
    type = single(zeros(1,5));
    path = RSPath_generate(type,0,0,0,0,0);
    [isok1,path1] = CSC(x,y,phi,RSPathElemType);
    [isok2,path2] = CCC(x,y,phi,RSPathElemType);
    [isok3,path3] = CCCC(x,y,phi,RSPathElemType);
    [isok4,path4] = CCSC(x,y,phi,RSPathElemType);
    [isok5,path5] = CCSCC(x,y,phi,RSPathElemType);
    isoks = [isok1, isok3, isok2, isok4, isok5];
    paths = [path1; path3; path2; path4; path5];
    Lmin = single(inf);
    for i = 1:5
        if isoks(i) == true
            elem = paths(i,:);
            if Lmin > elem(11)
                Lmin = elem(11);
                path = elem;
            end
        end
    end
end