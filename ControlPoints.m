classdef ControlPoints < handle
    %   ���ƶ����࣬�洢�ͼ������ڵ���ʸ��������
    
    properties
        controlpoints   %�洢���ƶ������ݵ�����
    end
    
    methods
        %grindingpointsΪһ�����飨Ԫ������ֻ���ĳһ��
        function obj=ControlPoints(VirtualSurface,GrindingPoints,order,CP_cnt)%���캯��
            initialKnots=ControlPoints.Getinitial(VirtualSurface,GrindingPoints,order,CP_cnt);%��ȡ��ʼֵ
            obj.controlpoints=initialKnots;%��ʼֵ
        end
        
        %�ı䵱ǰ���ƶ��������Ӧ�Ŀ��ƶ������ֵ
        function Changecontrolpoints(obj,newpoints)
            obj.controlpoints=newpoints;
        end
    end
    
    methods (Static=true)
        function InitialCP=Getinitial(VirtualSurface,GrindingPoints,order,CP_cnt)
            %orderΪ�����ߵĴ�����CP_cntΪ���Ƶ�ĸ���
            n=size(GrindingPoints,2);
            %�����㷨ʸ
            normal_vector = GetSurface_normal_vector( VirtualSurface,GrindingPoints);
            %�����������ʸ����һ������
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

