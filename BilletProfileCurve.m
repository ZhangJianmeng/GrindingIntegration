classdef BilletProfileCurve < handle
    %������� ���� �� �������� 
    %CSw = [deltaXw, deltaYw, deltaZw]   ��������ϵ��ԭ��W, �ڻ�������ϵ�µ�����
    properties
        Index      %ë�������ߵı��
        
        ProfilePoints    %�������ϵ���ɢ�㣬��3*N�ľ���ÿ�д洢һ���㡣ÿ�������ߴ洢N���㡣
        
        RemovedPoints    %���������ѱ�ĥ���ĵ㼯
        
        LeftPoints       %�����������µĵ㼯
        
    end
    
    methods
        %���캯��������ë���ֲ�Ĳ�ţ�ë���ڸò��������ߵĵ㼯
        function obj = BilletProfileCurve(ProfileIndex,Points)
            
        end
        
        %���ë������������ĥ��������
        function obj = SolveRemovedArea(WheelEnvelopeCurve)
            %�õ�ĥͷ��������δ��ĥ���ĵ㡣
            
        end
        
        %����ë�����������ѱ�ĥ���ĵ㼯������Ϊĥͷ�ڸò��ϵİ�����
        function RemovedPoints = GetRemovedPoints()
            %�õ�ĥͷ���������ѱ�ĥ���ĵ㡣
            
        end
        
        %����ë����������δ��ĥ���ĵ㼯������Ϊĥͷ�ڸò��ϵİ�����
        function LeftPoints = GetLeftPoints()
            %�õ�ĥͷ��������δ��ĥ���ĵ㡣
            
        end
               
    end
        
        
    end
    
end