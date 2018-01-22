classdef ControlPoints < handle
    %   控制顶点类，存储和计算用于刀轴矢量的数据
    
    properties
        controlpoints   %存储控制顶点数据的数组
    end
    
    methods
        %grindingpoints为一个数组（元胞），只针对某一行
        function obj=ControlPoints(VirtualSurface,GrindingPoints,order,CP_cnt)%构造函数
            initialKnots=ControlPoints.Getinitial(VirtualSurface,GrindingPoints,order,CP_cnt);%获取初始值
            obj.controlpoints=initialKnots;%初始值
        end
        
        %改变当前控制顶点类里对应的控制顶点的数值
        function Changecontrolpoints(obj,newpoints)
            obj.controlpoints=newpoints;
        end
    end
    
    methods (Static=true)
        function InitialCP=Getinitial(VirtualSurface,GrindingPoints,order,CP_cnt)
            %order为控制线的次数，CP_cnt为控制点的个数
            n=size(GrindingPoints,2);
            %刀触点法矢
            normal_vector = GetSurface_normal_vector( VirtualSurface,GrindingPoints);
            %刀触点粗略切矢，朝一个方向
            tangent_vector=zeros(3,n);
            for i=1:n-1
                tangent_vector(:,i)=GrindingPoints(:,i+1)-GrindingPoints(:,i);
            end
            tangent_vector(:,n)=tangent_vector(:,n-1);
            Secondary_normal_vector=zeros(3,n);
            Points=zeros(3,n);
            for i=1:n
                Secondary_normal_vector(:,i)=cross(tangent_vector(:,i),normal_vector(:,i))/norm(cross(tangent_vector(:,i),normal_vector(:,i)));
                Points(:,i)=GrindingPoints(:,i)+10*Secondary_normal_vector(:,i);
            end
            InitialCP =Approach_ControlPoints_basedPoints(Points,order,CP_cnt);
            
        end
    end
end

