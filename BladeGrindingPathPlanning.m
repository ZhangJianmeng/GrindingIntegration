%% ʵ��ҶƬ��ĥ��·���滮
clear;
tic
%---------------��ʼ��ʼ������-------------------------
%��ʼ��: ������B,C�ύ���ڻ�������ϵ�е����꣩
deltX = 0;
deltY = 0;
deltZ = 77.5;
machine = FiveAxisMachine(deltX, deltY, deltZ);
%��ʼ��: Բ��ɰ��
R = 5;
L = 20;
cutter = FilletGrindingWheel(R,L);
%��ʼ����ҶƬ��������������ϵ��ԭ��W�ڻ�������ϵ�µ����꣩
deltaXw = -13.123;
deltaYw = 3.631;
deltaZw = 42.5;
Z_Max = 100;   %Ҷ��ֲ�����Zֵ
Z_Min = 10;    %Ҷ��ֲ����СZֵ
Z_Group = linspace(Z_Min,Z_Max,100);%������зֲ��Zֵ

%�������������󣬹���ë���������
BilletSurface;
DesignSurface;

%�������������ë������Ķ���
Billet = Workpiece(deltaXw, deltaYw,deltaZw,Z_Group,BilletSurface);
Design = Workpiece(deltaXw, deltaYw,deltaZw,Z_Group,DesignSurface);

%---------------��ʼ����ĥ��·���滮ģ��--------------------------
%��ë�������ϲ������⵶λ�㣬�����к�����֯��λ������ľ���
m = 100;    %U�����õ�����ĥ����
n = 100;    %V�����õ�����ĥ����
U_para = linspace(0,1,m);
V_para = linspace(0,1,n);

VirtualGrindingPoints = Billet.PlanningVirtualGrindingPoints(U_para,V_para);%�������ĥ����

%����ĥ����
e = 0.02;
%���ù���ֵ
tol = 0.05;
%��õ��������꼯��
GrindingPoints = workpiece.GrindingPoints(e,Designsurface);


%����
for i = 1:m
    for j = 1:n
        Location_group = WheelLocationGroup();
        Location = WheelLocation(GrindingPoints(i,j));

    end
end

%���ë�������ڸ��ֲ��ϵ�������
BoundaryOfBilletOnLayer = Billet.BladeBoundaryOnLayer();
