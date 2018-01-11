function [nurbs] = GetNurbsSpline( Points,k )
%输入次数和物理空间中的点，运算得到一条三次准均匀B样条曲线
%Points为三维/二维数组，满足2（3）*N
%该函数为基础函数，无需进行修改
n=length(Points(1,:));
%% 节点矢量的配置
delt_u=1/(n-k);
mid=0:delt_u:1;
start=zeros(1,k);
knot_end=ones(1,k);
knot_u=[start,mid,knot_end];% knot_u=zeros(1,n+k+1);
u=aveknt(knot_u,k+1);%保证为准均匀样条
%% 反算控制顶点
m=n-1;%控制顶点数目-1
u=roundn(u,-10);
s = findspan (m, k, u, knot_u);
B = basisfun (s, u, k, knot_u);
t=s-(k-1);%目的是要求基函数是从第几个开始的
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+k)=B(i,:);
end
P=pinv(N)*Points';%求解;
nurbs=nrbmak(P',knot_u);%nurbs表示一个结构体，里面包含插值曲线的节点矢量、控制顶点、次数等；
end
