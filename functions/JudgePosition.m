function [ p_flag ] = JudgePosition( check_p,pointA,pointB )
%JUDGEPOISITION  �ж���һ��ƽ���ڵ���ֱ�ߵ���һ��
%   �жϱ�����check_pλ��point1��point2�����ߵ���໹���Ҳ�
%   ��Ϊ��࣬��Ϊ��,�Ҳ���Ϊ��
%   ������Ϊ3*1����ʽ����ֻ����x,y����ΪĬ��z����ͬ�ģ�����ͬһƽ��

A_Check=check_p-pointA;%Aָ��P��ʸ��
B_Check=check_p-pointB;%Bָ��P��ʸ��
Cross_P=cross(A_Check,B_Check);%����

if Cross_P(3)<0
    p_flag=-1;%�����
elseif Cross_P(3)>0
    p_flag=1;%���ұ�
else
    p_flag=0;
    fprintf('The test point is on the line.\n');
end

end

