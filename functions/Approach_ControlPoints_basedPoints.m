function [ CPoints ] = Approach_ControlPoints_basedPoints(Points,order,n )
%此函数用于获得逼近样条的控制顶点
%输入为参数点，样条次数，控制顶点的个数
%% 数据点的参数化
u=Ask_Points_U( Points );
%% 配置节点矢量
m= length (Points); %Points为N*3或者N*2的数组
knot_u(1,:)=linspace(0,0,order+1);%前端点的节点矢量重复度为order
Sum_ku = 0;
for j=2:n-order
    for i=j:m-n+order+j
        Sum_ku=Sum_ku+u(i);
    end
   knot_u(1,order+j)=(1/(m-n+1+order))*Sum_ku;
   Sum_ku=0;
end
for k1=1:order+1
    knot_u(1,n+k1)=1;
end
Knot_vector(1,:) = knot_u;
%% 求b样条基函数在参数点处的值
u=roundn(u,-6);%圆整
%% 求拟合曲线的b样条控制顶点
s = findspan (n-1, order, u, Knot_vector);
B = basisfun (s, u, order, Knot_vector); %基函数矩阵

R=B'*Points';

N =B'*B;

CPoints=N\R;
 
end