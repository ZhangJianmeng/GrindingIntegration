function U = U_GrindingPoints( GrindingPoints )
%利用累加弦长法求刀触点的参数
%输入一行刀触点，返回对应U参数组
% GrindingPoints=[4,3,3;4,3,5;5,0,4];
n=size(GrindingPoints,2);
dis=zeros(1,n-1);
parfor i=1:n-1
    dis(i)=norm(GrindingPoints(:,i+1)-GrindingPoints(:,i));%两点之间距离
end
Sumdis=sum(dis);
U=zeros(1,n);
for i=1:n-2
    U(i+1)=U(i)+norm(GrindingPoints(:,i+1)-GrindingPoints(:,i))/Sumdis;
end
U(length(U))=1;
end

