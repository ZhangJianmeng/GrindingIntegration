function [close0_u11,min_distance]=Askpoints_surMindis_total(Surface1,m0_points,M11,M22)
%获得面上一点的u参和v参
%M11和M22分别为u向和v向离散的点个数
ut=linspace(0,1,M22);
vt=linspace(0,1,M11);
[p,~] = nrbeval(Surface1,{ut,vt});
for j=1:length(m0_points(1,:))
    for ii=1:M11
        for i=1:M22
            distance(ii,i)=sqrt((m0_points(1,j)-p(1,i,ii))^2+(m0_points(2,j)-p(2,i,ii))^2+(m0_points(3,j)-p(3,i,ii))^2);
        end
    end
    [min_distance(j,1),index]=min(distance(:));
    [index_x,index_y]=ind2sub(size(distance),index);
    min_u(1,j)=ut(1,index_y); %最近点对应的u参合和v参
    min_v(1,j)=vt(1,index_x);
%     min_points(:,j)=p(:,index_y,index_x);
    
end
close0_u11=[min_u;min_v];
end

