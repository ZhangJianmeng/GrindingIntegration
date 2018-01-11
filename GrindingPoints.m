classdef GrindingPoints < handle
    %   此类用于表示刀触点和刀触点所在的虚拟曲面SV
    %   虽然可以将刀触点放在工件类里，但为了调整方便，将其整合为一个类
    
    properties
        Workpiece      %毛坯曲面，为类(Workpiece(Billet
        Design         %设计曲面，为类(Design
        TouchPoints    %加工所用的刀触点
        VirtualSurface %刀触点所在的虚拟曲面
    end
    
    methods
        function obj=GrindingPoints(Workpiece,Design)%构造函数,输入毛坯、设计
            obj.Workpiece=Workpiece;
            obj.Design=Design;
            
            %取较密的点来得到所有实际刀触点所对应的曲面，即虚拟曲面VirtualSurface，同样是分成两步
            
            %通过函数调用构造相应的虚拟刀触点(为元胞
            VirtualGrindingPoints=GrindingPoints.PlanningVirtualGrindingPoints(Workpiece.BoundaryGroups);
            
            %根据虚拟刀触点得到实际的刀触点
            obj.TouchPoints=GrindingPoints.GetAllGrindingPoints(...
                VirtualGrindingPoints,Design.BoundaryGroups,Design.Tolerance,Design.ZGroups);
            
            %根据实际的刀触点得到虚拟曲面
            obj.VirtualSurface=GrindingPoints.GetVirtualSurface(obj.TouchPoints);
        end
        
        %一共有n层（这个由设计曲面等决定，返回的是一行的,n表示对刀触点进行筛选，假设原本有1000个点，不可能把这个1000个点作为刀触点
        %m为隔几个挑，如果m为0的话，那么就返回所有的
        function GrindingPoints_Select=GetGrindingPoints(obj,index,m)
            CurrentGrindingPoints=cell2mat(obj.TouchPoints(index));
            n=length(CurrentGrindingPoints);
            GrindingPoints_Select=zeros(3,1);
            cnt=1;
            i=1;
            while i<n+1
                GrindingPoints_Select(:,cnt)=CurrentGrindingPoints(:,i);
                i=i+m+1;
                cnt=cnt+1;
            end
        end
    end
    
    methods (Static=true)
        %获取虚拟刀触点，用于计算得到实际刀触点
        %BoundaryGroups为一个元胞
        function VirtualGrindingPoints = PlanningVirtualGrindingPoints(BoundaryGroups)
            m=length(BoundaryGroups);
            VirtualGrindingPoints=cell(1,m);
            %每一行取极密,取1001个点
            parfor i=1:1:m
                nurbs=cell2mat(BoundaryGroups(i));
                Points=nrbeval(nurbs,0:0.001:1);
                VirtualGrindingPoints{1,i}=Points;
            end
        end
        
        %根据虚拟刀触点与设计曲面在每层的线的关系来得到实际刀触点
        %VirtualPoints为元胞
        %返回得到的切触点同样为元胞
        function TrueGrindingPoints=GetAllGrindingPoints(VirtualPoints,DesignGroups,Tolerance,Zgroup)
            n=size(VirtualPoints,2);%此时虚拟点有n层
            TrueGrindingPoints=cell(1,n);
            parfor i=1:1:n
                %对每层进行计算
                currentVirtualPoints=cell2mat(VirtualPoints(i));%该行的
                spline=cell2mat(DesignGroups(i));%目标曲线
                [~,CheckPoints]=Traversal(currentVirtualPoints,spline,400,[0,1]);
                %这里已经用parfor了，但根据测试，parfor里可以套用利用了parfor的function
                TrueGrindingPoints{1,i}=DecidePosition(CheckPoints,currentVirtualPoints,Zgroup(i),Tolerance);
            end
        end
        
        %根据点拟合得到曲面VirtualSurface
        function VirtualSurface=GetVirtualSurface(All_GrindingPoints)
            n=size(All_GrindingPoints,2);
            Nurbs=cell(1,n);
            parfor i=1:1:n%并行运算提高速度
                points=cell2mat(All_GrindingPoints(i));
                Nurbs{1,i}=GetNurbsSpline(points,3);
            end
            %蒙面法求解曲面，该Make_loftsurfaceKnow函数已经被调整为通用性质
            VirtualSurface=Make_loftsurfaceKnow(Nurbs);
        end
    end
    
end

