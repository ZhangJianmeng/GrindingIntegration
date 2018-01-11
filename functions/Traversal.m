function [Close_u,Q] = Traversal( m_points,Newtline_rotnew,N,um)
%m_points求解距离的点，要求为3*n，Newtline_rotnew为目标曲线，N求初始值离散点的数量，um在哪一块区域，为数组，如[0,1]，[0.2,0.4]
%用遍历的方法求牛顿迭代法最近点的初始参数值
%Q为最近点坐标，Close_u为参数
 t=linspace(um(1),um(end),N);
% t=linspace(0,1,N);
[q,~] = nrbeval(Newtline_rotnew,t);
% [u] = u_parame(q);
% m = length(m_points);
distance=zeros(1,N);
min_distance=zeros(length(m_points),1);
min_u=zeros(1,size(m_points,2));%按列赋值
for j=1:length(m_points)
    parfor i=1:N%个人修改，便于提高运算速度
        distance(:,i)=sqrt((m_points(1,j)-q(1,i))^2+(m_points(2,j)-q(2,i))^2+(m_points(3,j)-q(3,i))^2);
    end
    [min_distance(j,1),index]=min(distance(:,:));
    min_u(1,j)= t(1,index);
end
%牛顿迭代法求得最近点
[Close_u(1,:) ] = New_AskU( min_u,m_points,Newtline_rotnew );
[Q,~] = nrbeval(Newtline_rotnew,Close_u);
end
