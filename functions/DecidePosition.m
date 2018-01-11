function [ CheckingPoints ] = DecidePosition( CheckingPoints,VirtualPoints,Z_Value,Tolerance )
%DecidePosition �����ж��Ƿ����빫����㵶����
%   ë����������Ѿ����������ɵ���һ���֣�λ���¹������
%   ���ڴˣ���Ҫ�ж�ʵ�ʵ�������ë�����ߵ�λ�ù�ϵ
%   GrindingPoints��ʱΪVirtualPoints����������϶�Ӧ������㣬Z_ValueΪ��Ӧ��ֵ��ToleranceΪ����
n=size(CheckingPoints,2);%����

%�������������ж�
CheckingPoints(3,n)=Z_Value;
VirtualPoints(3,n)=Z_Value;
VirtualPoints(3,n-1)=Z_Value;
p_flag=JudgePosition(CheckingPoints(:,1),VirtualPoints(:,1),VirtualPoints(:,2));
if (p_flag>0)%���ұߣ����ϻ�������
    %��ô��ζ�Ÿô���ë��λ����Ƶ��¹����
    %��ȡ��Ƶ�ָ��ë�����ʸ��
    derive=VirtualPoints(:,1)-CheckingPoints(:,1);
    %�ظ÷��򣬶Ծ�������޸�
    d=(Tolerance-norm(derive))/2;
    vector=derive/norm(derive);
    CheckingPoints(:,1)=VirtualPoints(:,1)+vector*d;%���Ÿ�ʸ�������һ�����룬��֤���¹���
end

%�޷����в��м���
for i=1:1:n-1%��ÿ�������жϣ��жϷ������������ͬ
    CheckingPoints(3,i)=Z_Value;
    VirtualPoints(3,i)=Z_Value;
    VirtualPoints(3,i+1)=Z_Value;
    p_flag=JudgePosition(CheckingPoints(:,i),VirtualPoints(:,i),VirtualPoints(:,i+1));
    if (p_flag>0)%���ұߣ����ϻ�������
        %��ô��ζ�Ÿô���ë��λ����Ƶ��¹����
        %��ȡ��Ƶ�ָ��ë�����ʸ��
        derive=VirtualPoints(:,i)-CheckingPoints(:,i);
        %�ظ÷��򣬶Ծ�������޸�
        d=(Tolerance-norm(derive))/2;
        vector=derive/norm(derive);
        CheckingPoints(:,i)=VirtualPoints(:,i)+vector*d;%���Ÿ�ʸ�������һ�����룬��֤���¹���
    end
end

end

