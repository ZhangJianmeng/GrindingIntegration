function [Close_u,Q] = Traversal( m_points,Newtline_rotnew,N,um)
%m_points������ĵ㣬Ҫ��Ϊ3*n��Newtline_rotnewΪĿ�����ߣ�N���ʼֵ��ɢ���������um����һ������Ϊ���飬��[0,1]��[0.2,0.4]
%�ñ����ķ�����ţ�ٵ����������ĳ�ʼ����ֵ
%QΪ��������꣬Close_uΪ����
 t=linspace(um(1),um(end),N);
% t=linspace(0,1,N);
[q,~] = nrbeval(Newtline_rotnew,t);
% [u] = u_parame(q);
% m = length(m_points);
distance=zeros(1,N);
min_distance=zeros(length(m_points),1);
min_u=zeros(1,size(m_points,2));%���и�ֵ
for j=1:length(m_points)
    parfor i=1:N%�����޸ģ�������������ٶ�
        distance(:,i)=sqrt((m_points(1,j)-q(1,i))^2+(m_points(2,j)-q(2,i))^2+(m_points(3,j)-q(3,i))^2);
    end
    [min_distance(j,1),index]=min(distance(:,:));
    min_u(1,j)= t(1,index);
end
%ţ�ٵ�������������
[Close_u(1,:) ] = New_AskU( min_u,m_points,Newtline_rotnew );
[Q,~] = nrbeval(Newtline_rotnew,Close_u);
end
