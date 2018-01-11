function [ CPoints ] = ControlPoints_basedPoints( Points,order )

n=length(Points(1,:));
%% �ڵ�ʸ��������
delt_u=1/(n-order);
mid=0:delt_u:1;
start=zeros(1,order);
knot_end=ones(1,order);
knot_u=[start,mid,knot_end];% knot_u=zeros(1,n+order+1);
u=aveknt(knot_u,order+1);%��֤Ϊ׼��������
%% ������ƶ���
m=n-1;%���ƶ�����Ŀ-1
u=roundn(u,-10);
s = findspan (m, order, u, knot_u);
B = basisfun (s, u, order, knot_u);
t=s-(order-1);%Ŀ����Ҫ��������Ǵӵڼ�����ʼ��
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+order)=B(i,:);
end
CPoints=pinv(N)*Points';%���;

end

