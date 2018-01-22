 classdef WheelLocationGroup < handle
    %���������������Sv��һ�е����㼰���Ӧ�ĵ���ʸ����
    %�����Ӧ�ĵ�λ�㣨�Թ�������ϵΪ��׼��
    properties
        Sv;       %�������Sv
        GrindingPoints;     %ͬһ���ڵĵ���������
        GrindingWheel;       %ɰ��ģ��
    end
    
    methods
        %���캯��
        function obj = WheelLocationGroup(Sv,GrindingPoints,GrindingWheel)
            obj.Sv=Sv;
            obj.GrindingPoints=GrindingPoints;
            obj.GrindingWheel=GrindingWheel;
        end
        
        function Locations=GetWheelLocations(obj,VectorofWheelaxis)%VectorofWheelaxisΪ����ʸ��
            
            n=size(VectorofWheelaxis,2);
            %�õ����淨ʸ
            normal_vectors=GetSurface_normal_vector(obj.Sv,obj.GrindingPoints);
            for i=2:1:size(normal_vectors,2)
                if (dot(normal_vectors(:,i),normal_vectors(:,i-1))<0)
                    normal_vectors(:,i)=-normal_vectors(:,i);
                end
            end
            %%����Schmidt������������ͨ������ʸ���뷨ʸ���ɵ�ƽ�����뵶��ʸ����ֱ��ʸ��
            v=zeros(3,n);
            for i=1:n
                v(:,i)=normal_vectors(:,i)-dot(normal_vectors(:,i),VectorofWheelaxis(:,i))/dot(VectorofWheelaxis(:,i),VectorofWheelaxis(:,i))*VectorofWheelaxis(:,i);
                v(:,i)=v(:,i)/norm(v(:,i));
            end
            %����ʸ���ķ�����֪��ʸ����ӣ���L/2
            Locations=obj.GrindingPoints+(obj.GrindingWheel.L/2)*VectorofWheelaxis;
            %����֤�������������õ����뵶��ʸ����ֱ��ʸ������Ҷ��Ҷ���в��죬ͨ��������
            %�뷨ʸ�ļн��ж�R�ļӼ�
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

