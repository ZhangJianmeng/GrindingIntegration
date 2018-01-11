classdef Design < handle
    %   此类包含设计模型的基本参数
    
    properties
        %初始毛坯的参数
        %坐标原点也为手动输入
        deltaXw
        deltaYw
        deltaZw
        %----------^-^----^-^----^-^----^-^----------%
        zOmega           %切屑的层高，也可用n层来划分
        BladeSurface     %该结构对应的曲面
        BoundaryGroups   %工件在所有层上的边界
        BoundaryPoints   %工件在所有层上的点,元胞形式，且有顺序
        ZGroups          %工件每层所在的z平面坐标
        Tolerance        %工件的轮廓公差,如果是设计曲面那么就有，毛坯的话就为0
    end
    
    
    methods
        %构造函数
        function obj=Design(Wx,Wy,Wz,n,BladeSurface,Tolerance)
            obj.zOmega=n;
            obj.deltaXw = Wx;
            obj.deltaYw = Wy;
            obj.deltaZw = Wz;
            obj.BladeSurface=BladeSurface;
            obj.Tolerance=Tolerance;
        end
        
        %Zmin最底层所在的z，Zmax最上层所在的z
        function SetBoundaryGroup(obj,Zmin,Zmax)
            Height=Zmin:(Zmax-Zmin)/obj.zOmega:Zmax;%初始值
            Height_V=0:1/obj.zOmega:1;
            obj.ZGroups=Height;
            obj.BoundaryGroups=Getsection(obj.BladeSurface,Height,Height_V,200);%直接返回样条
            obj.BoundaryPoints=cell(1,length(obj.BoundaryGroups));
            %将每个样条上取101个点，暂时存着，如果有用的话
            for i=1:1:length(obj.BoundaryGroups)
                nurbs=cell2mat(obj.BoundaryGroups(i));%将元胞的数组转化为对应的样条
                points=zeros(3,100);
                cnt=1;
                for u=0:0.01:1
                    points(:,cnt)=nrbeval(nurbs,u);
                    cnt=cnt+1;
                end
                obj.BoundaryPoints{1,i}=points;
            end
        end
    end
    
end

