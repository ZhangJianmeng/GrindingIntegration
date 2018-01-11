function [ New_Nurbs ] = Getsection(Total_Surface,Height,Height_para,n)
% 该函数用于确定曲面在Z平面处的曲线
% Height为真实物理世界中的Z值，Height_para是将其参数化后的初始值，可以由0到1划分
% n为取样数，之前为2000，现在修改为输入的数
% Height表示高度，即z值
k=3;%样条次数
u=linspace(0,1,n);
U_Vparameter=cell(1,length(Height));
New_Nurbs=cell(1,length(Height));
for i=1:length(Height)
    x0=[u;ones(1,length(u))*Height_para(i)];
    [True_v(:,:)]=Newton_section(Height(i),x0,Total_Surface);%牛顿迭代求解相应z值处的u,v参
    U_Vparameter{1,i}=(True_v);
end
for i=1:length(Height)
    [p,~]=nrbeval(Total_Surface,U_Vparameter{1,i});
    New_Nurbs{1,i}=GetNurbsSpline(p,k);%得到三次准均匀B样条曲线
end
end

