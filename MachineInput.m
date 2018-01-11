classdef MachineInput < handle
    %该类包含 五轴机床的 位移参数 [xM,yM,zM,A,C] 
    properties
        xM
        yM
        zM
        A
        C
    end
    
    methods
        function obj=MachineInput(xM,yM,zM,A,C)
            obj.xM = xM;
            obj.yM = yM;
            obj.zM = zM;
            obj.A = A;
            obj.C = C;
        end
    end
end

