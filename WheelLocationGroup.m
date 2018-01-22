 classdef WheelLocationGroup < handle
    %该类输入拟合曲面Sv，一行刀触点及其对应的刀轴矢量，
    %输出对应的刀位点（以工件坐标系为基准）
    properties
        Sv;       %拟合曲面Sv
        GrindingPoints;     %同一行内的刀触点数组
        GrindingWheel;       %砂轮模型
    end
    
    methods
        %构造函数
        function obj = WheelLocationGroup(Sv,GrindingPoints,GrindingWheel)
            obj.Sv=Sv;
            obj.GrindingPoints=GrindingPoints;
            obj.GrindingWheel=GrindingWheel;
        end
        
        function Locations=GetWheelLocations(obj,VectorofWheelaxis)%VectorofWheelaxis为刀轴矢量
            
            n=size(VectorofWheelaxis,2);
            %得到曲面法矢
            normal_vectors=GetSurface_normal_vector(obj.Sv,obj.GrindingPoints);
            for i=2:1:size(normal_vectors,2)
                if (dot(normal_vectors(:,i),normal_vectors(:,i-1))<0)
                    normal_vectors(:,i)=-normal_vectors(:,i);
                end
            end
            %%利用Schmidt正交化方法，通过刀轴矢量与法矢构成的平面内与刀轴矢量垂直的矢量
            v=zeros(3,n);
            for i=1:n
                v(:,i)=normal_vectors(:,i)-dot(normal_vectors(:,i),VectorofWheelaxis(:,i))/dot(VectorofWheelaxis(:,i),VectorofWheelaxis(:,i))*VectorofWheelaxis(:,i);
                v(:,i)=v(:,i)/norm(v(:,i));
            end
            %刀轴矢量的方向已知，矢量相加，加L/2
            Locations=obj.GrindingPoints+(obj.GrindingWheel.L/2)*VectorofWheelaxis;
            %经验证，正交化方法得到的与刀轴矢量垂直的矢量方向叶盆叶背有差异，通过测试其
            %与法矢的夹角判断R的加减
            for i=1:n
                if(dot(normal_vectors(:,i),v(:,i))<0)
                    Locations(:,i)=Locations(:,i)+obj.GrindingWheel.R1*v(:,i);
                else
                    Locations(:,i)=Locations(:,i)-obj.GrindingWheel.R1*v(:,i);
                end
            end 
        end      
    end    
end

