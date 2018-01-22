classdef FilletGrindingWheel < handle
    %该类包含 刀具 的 几何参数 半径R 和 刀长L
    properties
        R1  %具体含义见论文图5-1
        L
        R2
        GrindingWheelSurface
    end
    
    methods
        %构造函数
        function obj = FilletGrindingWheel(R1,R2,L)
            obj.R1 = R1;
            obj.R2=R2;
            obj.L = L;
            obj.GrindingWheelSurface=FilletGrindingWheel.ConstructSurface(R1,L);
        end
        
    end
    methods (Static=true)
        %构造磨头曲面
        function Surface=ConstructSurface(R1,L)
            Surface=R1+L;%测试语句
        end
    end
    
end

