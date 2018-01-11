function [ normal_vector ] = GetSurface_normal_vector( VirtualSurface,GrindingPoints )
%获得面上点的法矢,注意：求得的法矢都是朝向面内
%输入：拟合刀触点所得平面，单个刀触点或者一组刀触点
%输出：对应刀触点的一个或者一组法矢
M=800;%描述曲面上离散的疏密程度
[Total_close_u,~]=Askpoints_surMindis_total(VirtualSurface,GrindingPoints,M,M);%获得刀触点对应的u，v参
%[Total_Close_points] = nrbeval(VirtualSurface,Total_close_u);
Deriv_Surface11 = nrbderiv (VirtualSurface);  
[~,Deriv_points] = nrbdeval(VirtualSurface,Deriv_Surface11,Total_close_u);
normal_vector=zeros(3,size(GrindingPoints,2));
parfor i=1:size(GrindingPoints,2)
    normal_vector(:,i)=cross(Deriv_points{1}(:,i),Deriv_points{2}(:,i))/norm(cross(Deriv_points{1}(:,i),Deriv_points{2}(:,i)));       
end  
end

