function [ points ] = GetEllipsePoints( theta,M,t,Z_omega,R,B )
%GETELLIPSEPOINTS 绘制整个圆
%   M为相应的变化矩阵，theta为参数的范围，t为离散个数，Z_omega为毛坯切面高度,R为平底刀刀具半径
%  B为角度
points=zeros(4,t);
theta=linspace(theta(1),theta(2),t);
l=Z_omega-M(3,1)*(R*cos(B))-M(3,4);%这里暂时不考虑通用性
l=l/M(3,3);
for i=1:1:t
    value=[R*cos(theta(i));R*sin(theta(i));l;1];
    points(:,i)=M*value;
end

