%% 实现叶片的磨削路径规划
clear;
tic
%---------------开始初始化工作-------------------------
%初始化: 机床（B,C轴交点在机床坐标系中的坐标）
deltX = 0;
deltY = 0;
deltZ = 77.5;
machine = FiveAxisMachine(deltX, deltY, deltZ);
%初始化: 圆柱砂轮
R = 5;
L = 20;
cutter = FilletGrindingWheel(R,L);
%初始化：叶片工件（工件坐标系的原点W在机床坐标系下的坐标）
deltaXw = -13.123;
deltaYw = 3.631;
deltaZw = 42.5;
Z_Max = 100;   %叶身分层的最大Z值
Z_Min = 10;    %叶身分层的最小Z值
Z_Group = linspace(Z_Min,Z_Max,100);%求得所有分层的Z值

%构建设计曲面对象，构建毛坯曲面对象
BilletSurface;
DesignSurface;

%返回设计曲面与毛坯曲面的对象
Billet = Workpiece(deltaXw, deltaYw,deltaZw,Z_Group,BilletSurface);
Design = Workpiece(deltaXw, deltaYw,deltaZw,Z_Group,DesignSurface);

%---------------开始建立磨削路径规划模型--------------------------
%在毛坯曲面上布置虚拟刀位点，按照行和列组织刀位点坐标的矩阵
m = 100;    %U方向布置的虚拟磨削点
n = 100;    %V方向布置的虚拟磨削点
U_para = linspace(0,1,m);
V_para = linspace(0,1,n);

VirtualGrindingPoints = Billet.PlanningVirtualGrindingPoints(U_para,V_para);%获得虚拟磨削点

%设置磨削量
e = 0.02;
%设置公差值
tol = 0.05;
%获得刀触点坐标集合
GrindingPoints = workpiece.GrindingPoints(e,Designsurface);


%创建
for i = 1:m
    for j = 1:n
        Location_group = WheelLocationGroup();
        Location = WheelLocation(GrindingPoints(i,j));

    end
end

%获得毛坯曲面在各分层上的轮廓线
BoundaryOfBilletOnLayer = Billet.BladeBoundaryOnLayer();
