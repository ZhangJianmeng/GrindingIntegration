function U =Ask_Points_U( Points )
%�����ۼ��ҳ����󵶴���Ĳ���
%����һ�е����㣬���ض�ӦU������
% GrindingPoints=[4,3,3;4,3,5;5,0,4];
n=size(Points,2);
dis=zeros(1,n-1);
parfor i=1:n-1
    dis(i)=norm(Points(:,i+1)-Points(:,i));%����֮�����
end
Sumdis=sum(dis);
U=zeros(1,n);
for i=1:n-2
    U(i+1)=U(i)+norm(Points(:,i+1)-Points(:,i))/Sumdis;
end
U(length(U))=1;
end

