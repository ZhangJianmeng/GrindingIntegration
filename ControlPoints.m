classdef ControlPoints < handle
    %   ���ƶ����࣬�洢�ͼ������ڵ���ʸ��������
    
    properties
        controlpoints   %�洢���ƶ������ݵ�����
    end
    
    methods
        %grindingpointsΪһ�����飨Ԫ������ֻ���ĳһ��
        function obj=ControlPoints(VirtualSurface,GrindingPoints)%���캯��
            initialKnots=ControlPoints.Getinitial(VirtualSurface,GrindingPoints);%��ȡ��ʼֵ
            obj.controlpoints=initialKnots;%��ʼֵ
        end
        
        %�ı䵱ǰ���ƶ��������Ӧ�Ŀ��ƶ������ֵ
        function Changecontrolpoints(obj,newpoints)
            obj.controlpoints=newpoints;
        end
    end
    
    methods (Static=true)
        function initialKnots=Getinitial(VirtualSurface,GrindingPoints)
            n=size(GrindingPoints,2);
            mid=int16(n/2);
            normal_vector = GetSurface_normal_vector( VirtualSurface,GrindingPoints);
            
            %������ֵʱ ����ȷ���������ס�β���м���������ʸ����һ������
            tangent_vector(:,1)=GrindingPoints(:,2)-GrindingPoints(:,1);
            tangent_vector(:,2)=GrindingPoints(:,mid+1)-GrindingPoints(:,mid);
            tangent_vector(:,3)=GrindingPoints(:,n)-GrindingPoints(:,n-1);
            %�����������ķ�ʸ
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

