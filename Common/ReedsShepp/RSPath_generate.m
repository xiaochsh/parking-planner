function obj = RSPath_generate(type,t,u,v,w,x)
    obj = single(zeros(1,11));
    obj(1:5) = type;                          %type
    obj(6) = t;                               %t
    obj(7) = u;                            
    obj(8) = v;
    obj(9) = w;
    obj(10) = x;
    obj(11)= sum(abs([t,u,v,w,x]));  %totalLength
end
