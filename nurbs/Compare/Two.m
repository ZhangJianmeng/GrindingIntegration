function [ P,nurbs ] = Two( k,Points)
n=length(Points(1,:));
knot_u=zeros(1,n+k+1);
delt_u=1/(n-k);
mid=[0:delt_u:1];
start=zeros(1,k);
knot_end=ones(1,k);
knot_u=[start,mid,knot_end];
u=aveknt(knot_u,k+1);
m=n-1;%控制顶点数目-1
u=roundn(u,-10);
s = findspan (m, k, u, knot_u);
B = basisfun (s, u, k, knot_u);
t=s-(k-1);
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+k)=B(i,:);
end
P=transpose(pinv(N)*Points');
nurbs=nrbmak(P,knot_u);
end

