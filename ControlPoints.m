classdef ControlPoints < handle
    %   控制顶点类，存储和计算用于刀轴矢量的数据
    
    properties
        controlpoints   %存储控制顶点数据的数组
    end
    
    methods
        %grindingpoints为一个数组（元胞），只针对某一行
        function obj=ControlPoints(VirtualSurface,GrindingPoints)%构造函数
            initialKnots=ControlPoints.Getinitial(VirtualSurface,GrindingPoints);%获取初始值
            obj.controlpoints=initialKnots;%初始值
        end
        
        %改变当前控制顶点类里对应的控制顶点的数值
        function Changecontrolpoints(obj,newpoints)
            obj.controlpoints=newpoints;
        end
    end
    
    methods (Static=true)
        function initialKnots=Getinitial(VirtualSurface,GrindingPoints)
            n=size(GrindingPoints,2);
            mid=int16(n/2);
            normal_vector = GetSurface_normal_vector( VirtualSurface,GrindingPoints);
            
            %给定初值时 粗略确定刀触点首、尾、中间三处的切矢，朝一个方向
            tangent_vector(:,1)=GrindingPoints(:,2)-GrindingPoints(:,1);
            tangent_vector(:,2)=GrindingPoints(:,mid+1)-GrindingPoints(:,mid);
            tangent_vector(:,3)=GrindingPoints(:,n)-GrindingPoints(:,n-1);
            %获得以上三点的法矢
            normal_vector(:,1) = GetSurface_normal_vector( VirtualSurface,GrindingPoints(:,1));
            normal_vector(:,2) = GetSurface_normal_vector( VirtualSurface,GrindingPoints(:,mid));
            normal_vector(:,3) = GetSurface_normal_vector( VirtualSurface,GrindingPoints(:,n));
            
            Secondary_normal_vector=zeros(3:3);
            
            parfor i=1:3
                Secondary_normal_vector(:,i)=cross(tangent_vector(:,i),normal_vector(:,i))/norm(cross(tangent_vector(:,i),normal_vector(:,i)));
            end
            Points(:,1)=GrindingPoints(:,1)+Secondary_normal_vector(:,1);
            Points(:,2)=GrindingPoints(:,mid-1)+Secondary_normal_vector(:,2);
            Points(:,3)=GrindingPoints(:,n)+Secondary_normal_vector(:,3);
            initialKnots = ControlPoints_basedPoints( Points,2);
        end
    end
end

