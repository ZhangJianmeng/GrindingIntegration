%提供原始数据，该数据已经存储在datas.mat中，不需修改
clear all;clc;
%确定机床
detP=[0 0 100];%枢轴点在机床坐标系中的坐标（机床固有）
detW=[10 10 200];%加工坐标系原点在机床坐标系中的坐标（通过对刀得到）
L=50;%刀长（刀补后）
machine=FiveAxisMachine(detP,detW,L);

%确定刀位点1
Point_X1=100;%测量点或者刀位点在加工坐标系中的坐标
Point_Y1=300;
Point_Z1=300;
Middle_Vector=[0 0 1]; %刀轴矢量在加工坐标系中的表示
Middle_Vector=Middle_Vector/norm(Middle_Vector);
Nx=Middle_Vector(1);%x分量
Ny=Middle_Vector(2);%y分量
Nz=Middle_Vector(3);%z分量
Machinginput1 = machine.ComputePoisition(Point_X1,Point_Y1,Point_Z1,Nx,Ny,Nz );

%确定刀位点2
Point_X2=95;%测量点或者刀位点在加工坐标系中的坐标
Point_Y2=310;
Point_Z2=300;
Middle_Vector2=[0 0 1]; %刀轴矢量在加工坐标系中的表示
Middle_Vector2=Middle_Vector2/norm(Middle_Vector2);
Nx2=Middle_Vector2(1);%x分量
Ny2=Middle_Vector2(2);%y分量
Nz2=Middle_Vector2(3);%z分量
Machinginput2= machine.ComputePoisition(Point_X2,Point_Y2,Point_Z2,Nx2,Ny2,Nz2 );
%确定刀具
cutter=FilletGrindingWheel(10,0,30);
%确定工作机床
MyWorkingMachine=WorkingMachine(machine,cutter);
%计算这两个刀位点之间的关系
Machingposition=MachingPosition(Machinginput1,Machinginput2,MyWorkingMachine,machine);
