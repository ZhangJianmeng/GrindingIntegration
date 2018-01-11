classdef FilletGrindingWheel < handle
    %������� ���� �� ���β��� �뾶R �� ����L
    properties
        R1  %���庬�������ͼ5-1
        L
        GrindingWheelSurface
    end
    
    methods
        %���캯��
        function obj = GrindingWheel(R1,L)
            obj.R1 = R1;
            obj.L = L;
            obj.GrindingWheelSurface=FilletGrindingWheel.ConstructSurface(R1,L);
        end
        
    end
    methods (Static=true)
        %����ĥͷ����
        function Surface=ConstructSurface(R1,L)
            Surface=R1+L;%�������
        end
    end
    
end

