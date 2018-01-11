classdef GrindingPoints < handle
    %   �������ڱ�ʾ������͵��������ڵ���������SV
    %   ��Ȼ���Խ���������ڹ��������Ϊ�˵������㣬��������Ϊһ����
    
    properties
        Workpiece      %ë�����棬Ϊ��(Workpiece(Billet
        Design         %������棬Ϊ��(Design
        TouchPoints    %�ӹ����õĵ�����
        VirtualSurface %���������ڵ���������
    end
    
    methods
        function obj=GrindingPoints(Workpiece,Design)%���캯��,����ë�������
            obj.Workpiece=Workpiece;
            obj.Design=Design;
            
            %ȡ���ܵĵ����õ�����ʵ�ʵ���������Ӧ�����棬����������VirtualSurface��ͬ���Ƿֳ�����
            
            %ͨ���������ù�����Ӧ�����⵶����(ΪԪ��
            VirtualGrindingPoints=GrindingPoints.PlanningVirtualGrindingPoints(Workpiece.BoundaryGroups);
            
            %�������⵶����õ�ʵ�ʵĵ�����
            obj.TouchPoints=GrindingPoints.GetAllGrindingPoints(...
                VirtualGrindingPoints,Design.BoundaryGroups,Design.Tolerance,Design.ZGroups);
            
            %����ʵ�ʵĵ�����õ���������
            obj.VirtualSurface=GrindingPoints.GetVirtualSurface(obj.TouchPoints);
        end
        
        %һ����n�㣨������������Ⱦ��������ص���һ�е�,n��ʾ�Ե��������ɸѡ������ԭ����1000���㣬�����ܰ����1000������Ϊ������
        %mΪ�������������mΪ0�Ļ�����ô�ͷ������е�
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
        %��ȡ���⵶���㣬���ڼ���õ�ʵ�ʵ�����
        %BoundaryGroupsΪһ��Ԫ��
        function VirtualGrindingPoints = PlanningVirtualGrindingPoints(BoundaryGroups)
            m=length(BoundaryGroups);
            VirtualGrindingPoints=cell(1,m);
            %ÿһ��ȡ����,ȡ1001����
            parfor i=1:1:m
                nurbs=cell2mat(BoundaryGroups(i));
                Points=nrbeval(nurbs,0:0.001:1);
                VirtualGrindingPoints{1,i}=Points;
            end
        end
        
        %�������⵶���������������ÿ����ߵĹ�ϵ���õ�ʵ�ʵ�����
        %VirtualPointsΪԪ��
        %���صõ����д���ͬ��ΪԪ��
        function TrueGrindingPoints=GetAllGrindingPoints(VirtualPoints,DesignGroups,Tolerance,Zgroup)
            n=size(VirtualPoints,2);%��ʱ�������n��
            TrueGrindingPoints=cell(1,n);
            parfor i=1:1:n
                %��ÿ����м���
                currentVirtualPoints=cell2mat(VirtualPoints(i));%���е�
                spline=cell2mat(DesignGroups(i));%Ŀ������
                [~,CheckPoints]=Traversal(currentVirtualPoints,spline,400,[0,1]);
                %�����Ѿ���parfor�ˣ������ݲ��ԣ�parfor���������������parfor��function
                TrueGrindingPoints{1,i}=DecidePosition(CheckPoints,currentVirtualPoints,Zgroup(i),Tolerance);
            end
        end
        
        %���ݵ���ϵõ�����VirtualSurface
        function VirtualSurface=GetVirtualSurface(All_GrindingPoints)
            n=size(All_GrindingPoints,2);
            Nurbs=cell(1,n);
            parfor i=1:1:n%������������ٶ�
                points=cell2mat(All_GrindingPoints(i));
                Nurbs{1,i}=GetNurbsSpline(points,3);
            end
            %���淨������棬��Make_loftsurfaceKnow�����Ѿ�������Ϊͨ������
            VirtualSurface=Make_loftsurfaceKnow(Nurbs);
        end
    end
    
end

