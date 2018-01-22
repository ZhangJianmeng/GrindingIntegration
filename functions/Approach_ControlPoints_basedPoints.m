function [ CPoints ] = Approach_ControlPoints_basedPoints(Points,order,n )
%�˺������ڻ�ñƽ������Ŀ��ƶ���
%����Ϊ�����㣬�������������ƶ���ĸ���
%% ���ݵ�Ĳ�����
u=Ask_Points_U( Points );
%% ���ýڵ�ʸ��
m= length (Points); %PointsΪN*3����N*2������
knot_u(1,:)=linspace(0,0,order+1);%ǰ�˵�Ľڵ�ʸ���ظ���Ϊorder
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
%% ��b�����������ڲ����㴦��ֵ
u=roundn(u,-6);%Բ��
%% ��������ߵ�b�������ƶ���
s = findspan (n-1, order, u, Knot_vector);
B = basisfun (s, u, order, Knot_vector); %����������

R=B'*Points';

N =B'*B;

CPoints=N\R;
 
end