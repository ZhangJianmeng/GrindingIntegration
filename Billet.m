classdef Billet < handle
    %   �������ë���Ļ�������
    
    properties
        
        %��ʼë���Ĳ���
        %����ԭ��ҲΪ�ֶ�����
        deltaXw
        deltaYw
        deltaZw
        %----------^-^----^-^----^-^----^-^----------%
        zOmega           %��м�Ĳ�ߣ�Ҳ����n��������
        BladeSurface     %�ýṹ��Ӧ������
        BoundaryGroups   %���������в��ϵı߽�
        BoundaryPoints   %���������в��ϵĵ�,Ԫ����ʽ������˳��
        ZGroups          %����ÿ�����ڵ�zƽ������
    end
    
    
    methods
        %���캯��
        function obj=Billet(Wx,Wy,Wz,n,BladeSurface)
            obj.zOmega=n;
            obj.deltaXw = Wx;
            obj.deltaYw = Wy;
            obj.deltaZw = Wz;
            obj.BladeSurface=BladeSurface;
        end
        
        %Zmin��ײ����ڵ�z��Zmax���ϲ����ڵ�z
        function SetBoundaryGroup(obj,Zmin,Zmax)
            Height=Zmin:(Zmax-Zmin)/obj.zOmega:Zmax;%��ʼֵ
            Height_V=0:1/obj.zOmega:1;
            obj.ZGroups=Height;
            obj.BoundaryGroups=Getsection(obj.BladeSurface,Height,Height_V,200);%ֱ�ӷ�������
            obj.BoundaryPoints=cell(1,length(obj.BoundaryGroups));
            %��ÿ��������ȡ101���㣬��ʱ���ţ�������õĻ�
            for i=1:1:length(obj.BoundaryGroups)
                nurbs=cell2mat(obj.BoundaryGroups(i));%��Ԫ��������ת��Ϊ��Ӧ������
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

