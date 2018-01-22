classdef MachingInput < handle
    %该类包含 五轴机床的 位移参数 [Xm,Ym,Zm,B,C] 
    %专门用于存储相应的值
    properties
        Xm
        Ym
        Zm
        B
        C
    end
    
    methods
        function obj=MachingInput(Xm,Ym,Zm,B,C)
            obj.Xm = Xm;
            obj.Ym = Ym;
            obj.Zm = Zm;
            obj.B = B;
            obj.C = C;
        end
    end
end

