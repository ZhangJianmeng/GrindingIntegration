classdef MachineInput < handle
    %������� ��������� λ�Ʋ��� [xM,yM,zM,A,C] 
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

