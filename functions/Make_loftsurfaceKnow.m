function [Surface]=Make_loftsurfaceKnow(Iteration_blade)
% Iteration_blade��һ��1XN��Ԫ�����洢��n��B��������
n=length(Iteration_blade);
origin_nurbs=cell(1,1,n);
for i=1:n
    origin_nurbs{:,:,i}=Iteration_blade{:,i};
end

%% �Խ����߽��нڵ�ϸ��,�ҳ�ÿ��������Ҫ����Ľڵ�
format long;
knots_sum=cell(1,1,n);
for i=1:n
    [m1,n1]=size(origin_nurbs{:,:,i}.knots);
    knots_total=ones(m1,n1,1);
    for j=1:n
        if(j==i)
            continue;
        else
            knots_total=union(knots_total,origin_nurbs{:,:,j}.knots);
        end
    end
    knots_sum{:,:,i}=knots_total;
    
end
knots_new=cell(1,1,i);
insert_knots=cell(1,1,i);
for i=1:n %�����������������������һ������
    knots_new{:,:,i}=union(origin_nurbs{:,:,i}.knots, knots_sum{:,:,i});
    insert_knots{:,:,i}=setdiff(knots_new{:,:,i},origin_nurbs{:,:,i}.knots);
end
%% 51�������߽ڵ�Ĳ���
all_inurbs=cell(1,n);
for i=1:1:n
    all_inurbs{:,i}=nrbkntins(origin_nurbs{:,:,i},insert_knots{:,:,i});
end
% ��ϸ�ֺ��b�������߷���һ��Ԫ����
nurbs_total=cell(1,n);
for i=1:1:n
    nurbs_one=cell2mat(all_inurbs(1,i));
    nurbs_total{1,i}=nurbs_one.coefs;
end

%% v�����Ͻ��в�ֵ(ʹ���ҳ��������ķ���)
%------------------------------v���������ݵ�Ĳ�����
% k=3; %v�����ϵĴ���
% n2=length(inurbs0.coefs);%���ƶ���ĸ���
% for i=1:n2
%     for j=1:n-1
%        distance(i,j)=norm((nurbs_total{:,:,j}.coefs(:,i)-nurbs_total{:,:,j+1}.coefs(:,i)));
%     end
% end
% d=sum(distance,2);%��ÿ�������������п��ƶ�������ҳ�
% v(1)=0;
% for i=2:n
%     sum_delt_p=0;
%     for j=1:n2
%         sum_delt_p=sum_delt_p+norm(nurbs_total{:,:,i}.coefs(:,j)-nurbs_total{:,:,i-1}.coefs(:,j))/d(j);
%     end
%     v(i)=v(i-1)+(1/n2)*sum_delt_p;
% end
%% v�����Ͻ��в�ֵ��ʹ�����Ĳ�������
k=3; %v�����ϵĴ���
nurbs_help=cell2mat(nurbs_total(1,1));
n2=length(nurbs_help);%���ƶ���ĸ���
distance=zeros(n2,n-1);
for i=1:n2
    for j=1:n-1
        distance(i,j)=sqrt(norm((nurbs_total{1,j}(:,i)-nurbs_total{1,j+1}(:,i))));
    end
end
d=sum(distance,2);%��ÿ�������������п��ƶ�������ҳ�
v=zeros(1,n);
for i=2:n
    sum_delt_p=0;
    for j=1:n2
        sum_delt_p=sum_delt_p+sqrt(norm(nurbs_total{1,i}(:,j)-nurbs_total{1,i-1}(:,j)))/d(j);
    end
    v(i)=v(i-1)+(1/n2)*sum_delt_p;
end
%-----------------------------------------------
%v�����Ͻڵ�ʸ��������(����ƽ������)
knot_v(1,:)=linspace(0,0,k+1);%ǰ�˵�Ľڵ�ʸ���ظ���Ϊ4
knot_v(1,n+1:n+k+1)=1;
for j=2:n-k
    sum_v=0;
    for i=j:j+k-1
        sum_v=sum_v+v(i);
    end
    knot_v(j+k)=(1/k)*sum_v;
end
%% ���������ϵĿ��ƶ���(����һ��һ����v�ϵĿ��ƶ���)
m=n-1;%v�����ϵĿ��ƶ�����Ŀ-1
v=roundn(v,-6);
s = findspan (m, k, v, knot_v);
B = basisfun (s, v, k, knot_v);
t=s-2;%Ŀ����Ҫ��������Ǵӵڼ�����ʼ��
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+3)=B(i,:);
end
% ---------------------------------------------
%����������ƶ���

for i=1:n2
    C(:,:)=zeros(3,n);
    %--------------------
    cc=[nurbs_total{1,:}];
    C(:,:)=cc(1:3,i:n2:end);
    
    D(1,i,:)=inv(N)*C(1,:)';
    D(2,i,:)=inv(N)*C(2,:)';
    D(3,i,:)=inv(N)*C(3,:)';
end

% ���õ�������
inurbs1=cell2mat(all_inurbs(1));
u_knots=inurbs1.knots;%u�����ϵĽڵ�ʸ��
vv=knot_v;
Surface=nrbmak(D,{u_knots,vv});
% read_knots(inurbs1,knot_v,D);
end

