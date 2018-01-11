classdef VectorofWheelaxis<handle
    %����ʸ���࣬�������������,�������ߵĴ���,���ƶ����䣬����Ϊ���캯����Ա�����ں���������
    %�������ʸ����
    properties
        GrindingPoints;%������
        order;%�����ߴ���
    end
    methods
        %GrindingPointsΪ3*N�ľ���order����2
        function obj=VectorofWheelaxis(GrindingPoints,order)
            obj.GrindingPoints=GrindingPoints;
            obj.order=order;
            
        end
        %ControlPoints�����Ϊһ������3*N
        function Vector=GetVectorofWheelaxis(obj,ControlPoints)%���ƶ���
            %1.�����ۼ��ҳ�����GringdingPoints��Ӧ�Ĳ�������U
            U=U_GrindingPoints(obj.GrindingPoints);
            %2.����ControlPoints�ʹ���order�õ�������Curve
            Curve = ControlCurve(ControlPoints,obj.order);
            %3.��Sc����ȡu(i)�õ���Ӧ��������GrindingPoints(i)�Ķ�Ӧ��Q��
            %�Ӷ�����ȷ������ʸ�����õ�����ʸ����
            Vector=zeros(3,length(U));
            parfor i=1:1:length(U)
                [Q,~]= nrbeval(Curve,U(i));
                Vector(:,i) =(obj.GrindingPoints(:,i)-Q)/norm(obj.GrindingPoints(:,i)-Q);
            end
        end
    end
end