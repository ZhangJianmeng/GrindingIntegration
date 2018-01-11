function [Surface]=Make_loftsurfaceKnow(Iteration_blade)
% Iteration_blade是一个1XN的元胞，存储了n条B样条曲线
n=length(Iteration_blade);
origin_nurbs=cell(1,1,n);
for i=1:n
    origin_nurbs{:,:,i}=Iteration_blade{:,i};
end

%% 对截面线进行节点细分,找出每条截面线要插入的节点
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
for i=1:n %这条截面线与其余截面线做一个并集
    knots_new{:,:,i}=union(origin_nurbs{:,:,i}.knots, knots_sum{:,:,i});
    insert_knots{:,:,i}=setdiff(knots_new{:,:,i},origin_nurbs{:,:,i}.knots);
end
%% 51条截面线节点的插入
all_inurbs=cell(1,n);
for i=1:1:n
    all_inurbs{:,i}=nrbkntins(origin_nurbs{:,:,i},insert_knots{:,:,i});
end
% 把细分后的b样条曲线放在一个元胞中
nurbs_total=cell(1,n);
for i=1:1:n
    nurbs_one=cell2mat(all_inurbs(1,i));
    nurbs_total{1,i}=nurbs_one.coefs;
end

%% v方向上进行插值(使用弦长参数化的方法)
%------------------------------v方向上数据点的参数化
% k=3; %v方向上的次数
% n2=length(inurbs0.coefs);%控制顶点的个数
% for i=1:n2
%     for j=1:n-1
%        distance(i,j)=norm((nurbs_total{:,:,j}.coefs(:,i)-nurbs_total{:,:,j+1}.coefs(:,i)));
%     end
% end
% d=sum(distance,2);%求每条截面线上所有控制顶点的总弦长
% v(1)=0;
% for i=2:n
%     sum_delt_p=0;
%     for j=1:n2
%         sum_delt_p=sum_delt_p+norm(nurbs_total{:,:,i}.coefs(:,j)-nurbs_total{:,:,i-1}.coefs(:,j))/d(j);
%     end
%     v(i)=v(i-1)+(1/n2)*sum_delt_p;
% end
%% v方向上进行插值（使用向心参数化）
k=3; %v方向上的次数
nurbs_help=cell2mat(nurbs_total(1,1));
n2=length(nurbs_help);%控制顶点的个数
distance=zeros(n2,n-1);
for i=1:n2
    for j=1:n-1
        distance(i,j)=sqrt(norm((nurbs_total{1,j}(:,i)-nurbs_total{1,j+1}(:,i))));
    end
end
d=sum(distance,2);%求每条截面线上所有控制顶点的总弦长
v=zeros(1,n);
for i=2:n
    sum_delt_p=0;
    for j=1:n2
        sum_delt_p=sum_delt_p+sqrt(norm(nurbs_total{1,i}(:,j)-nurbs_total{1,i-1}(:,j)))/d(j);
    end
    v(i)=v(i-1)+(1/n2)*sum_delt_p;
end
%-----------------------------------------------
%v方向上节点矢量的配置(采用平均技术)
knot_v(1,:)=linspace(0,0,k+1);%前端点的节点矢量重复度为4
knot_v(1,n+1:n+k+1)=1;
for j=2:n-k
    sum_v=0;
    for i=j:j+k-1
        sum_v=sum_v+v(i);
    end
    knot_v(j+k)=(1/k)*sum_v;
end
%% 反算曲面上的控制顶点(首先一次一次算v上的控制顶点)
m=n-1;%v方向上的控制顶点数目-1
v=roundn(v,-6);
s = findspan (m, k, v, knot_v);
B = basisfun (s, v, k, knot_v);
t=s-2;%目的是要求基函数是从第几个开始的
N=zeros(n,m+1);
for i=1:n
    N(i,t(i):t(i)+3)=B(i,:);
end
% ---------------------------------------------
%反算曲面控制顶点

for i=1:n2
    C(:,:)=zeros(3,n);
    %--------------------
    cc=[nurbs_total{1,:}];
    C(:,:)=cc(1:3,i:n2:end);
    
    D(1,i,:)=inv(N)*C(1,:)';
    D(2,i,:)=inv(N)*C(2,:)';
    D(3,i,:)=inv(N)*C(3,:)';
end

% 检测得到的曲面
inurbs1=cell2mat(all_inurbs(1));
u_knots=inurbs1.knots;%u方向上的节点矢量
vv=knot_v;
Surface=nrbmak(D,{u_knots,vv});
% read_knots(inurbs1,knot_v,D);
end

