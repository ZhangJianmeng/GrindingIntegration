classdef VectorofWheelaxis<handle
    %刀轴矢量类，输入包括刀触点,辅助曲线的次数,控制顶点多变，不作为构造函数成员，放在函数中输入
    %输出刀轴矢量组
    properties
        GrindingPoints;%刀触点
        order;%控制线次数
    end
    methods
        %GrindingPoints为3*N的矩阵，order建议2
        function obj=VectorofWheelaxis(GrindingPoints,order)
            obj.GrindingPoints=GrindingPoints;
            obj.order=order;
            
        end
        %ControlPoints输入的为一个数组3*N
        function Vector=GetVectorofWheelaxis(obj,ControlPoints)%控制顶点
            %1.根据累加弦长计算GringdingPoints对应的参数数组U
            U=Ask_Points_U(obj.GrindingPoints);
            %2.根据ControlPoints和次数order得到控制线Curve
            Curve = ControlCurve(ControlPoints,obj.order);
            %3.在Sc依次取u(i)得到对应刀触点组GrindingPoints(i)的对应点Q；
            %从而两点确定刀轴矢量，得到刀轴矢量组
            Vector=zeros(3,length(U));
            parfor i=1:1:length(U)
                [Q,~]= nrbeval(Curve,U(i));
                Vector(:,i) =(obj.GrindingPoints(:,i)-Q)/norm(obj.GrindingPoints(:,i)-Q);
            end
        end
    end
end