classdef FilletGrindingWheel < handle
    %������� ���� �� ���β��� �뾶R �� ����L
    properties
        R1;  %���庬�������ͼ5-1
        R2;  %Բ��ĥͷ��ʼ��Ϊ0
        L;
        GrindingWheelSurface
    end
    
    methods
        %���캯��
        function obj = GrindingWheel(R1,R2,L)
            obj.R1 = R1;
            obj.R2 = R2;
            obj.L = L;
            
            obj.GrindingWheelSurface=FilletGrindingWheel.ConstructSurface(R1,R2,L);
            
        end
        
    end
    methods (Static=true)
        %����ĥͷ����
        function Surface=ConstructSurface(R1,R2,L)
            Surface=R1+L+R2;%�������
        end
    end
    
end

