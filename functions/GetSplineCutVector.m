function [ points,deriv_vector ] = GetSplineCutVector( spline,u )
%GETSPLINECUTVECTOR ��ȡ�������߶�Ӧu������ʸ�͵�
%   splineΪ��������u����Ϊһ�����飬�洢��Ӧ�Ĳ���
all_vector=nrbderiv(spline);
[points,deriv_vector]=nrbdeval(spline,all_vector,u);%u��Ӧ�ĵ����������һ�㴦����ʸ
end

