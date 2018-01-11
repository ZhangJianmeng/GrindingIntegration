function [nurbs] = GetNurbsSpline( Points,k )
%�������������ռ��еĵ㣬����õ�һ������׼����B��������
%PointsΪ��ά/��ά���飬����2��3��*N
%�ú���Ϊ������������������޸�
n=length(Points(1,:));
%% �ڵ�ʸ��������
delt_u=1/(n-k);
mid=0:delt_u:1;
start=zeros(1,k);
knot_end=ones(1,k);
knot_u=[start,mid,knot_end];% knot_u=zeros(1,n+k+1);
u=aveknt(knot_u,k+1);%��֤Ϊ׼��������
%% ������ƶ���
m=n-1;%���ƶ�����Ŀ-1
u=roundn(u,-10);
s = findspan (m, k, u, knot_u);
B = basisfun (s, u, k, knot_u);
t=s-(k-1);%Ŀ����Ҫ��������Ǵӵڼ�����ʼ��
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+k)=B(i,:);
end
P=pinv(N)*Points';%���;
nurbs=nrbmak(P',knot_u);%nurbs��ʾһ���ṹ�壬���������ֵ���ߵĽڵ�ʸ�������ƶ��㡢�����ȣ�
end
