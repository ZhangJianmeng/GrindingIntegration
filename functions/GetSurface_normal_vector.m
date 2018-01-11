function [ normal_vector ] = GetSurface_normal_vector( VirtualSurface,GrindingPoints )
%������ϵ�ķ�ʸ,ע�⣺��õķ�ʸ���ǳ�������
%���룺��ϵ���������ƽ�棬�������������һ�鵶����
%�������Ӧ�������һ������һ�鷨ʸ
M=800;%������������ɢ�����̶ܳ�
[Total_close_u,~]=Askpoints_surMindis_total(VirtualSurface,GrindingPoints,M,M);%��õ������Ӧ��u��v��
%[Total_Close_points] = nrbeval(VirtualSurface,Total_close_u);
Deriv_Surface11 = nrbderiv (VirtualSurface);  
[~,Deriv_points] = nrbdeval(VirtualSurface,Deriv_Surface11,Total_close_u);
normal_vector=zeros(3,size(GrindingPoints,2));
parfor i=1:size(GrindingPoints,2)
    normal_vector(:,i)=cross(Deriv_points{1}(:,i),Deriv_points{2}(:,i))/norm(cross(Deriv_points{1}(:,i),Deriv_points{2}(:,i)));       
end  
end

