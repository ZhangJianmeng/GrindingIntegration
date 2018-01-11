function [ side_nurbs,otherside_nurbs ] = GetNurbSingleSide( nurbs,n )
%GETDIVIDENURB �������ʽ�һ����յ��������߻��ֳ���
%   �ú������ڰѷ�յ��������߻���ΪҶ���Ҷ������
%   �����nurbs����ж��������Ϊ���飬����ΪԪ��,nΪ�����n��u�������

%   ����ֵΪԪ����ʽ����������

side_nurbs=cell(1,length(nurbs));
otherside_nurbs=cell(1,length(nurbs));

for i=1:1:length(nurbs)
    u=0:1/n:1;
    %��ȡ���u�ζ�Ӧ�������ϵĵ����������һ�㴦����ʸ
    [points,deriv_vector]=GetSplineCutVector(nurbs(i),u);
    rands=zeros(1,length(deriv_vector));
    for j=1:1:size(points,2)-1
        a=deriv_vector(:,j);
        b=deriv_vector(:,j+1);
        rands(j)=dot(a,b)/norm((a)*norm(b));%������������ʸ֮��Ľ���
    end
    side_u=zeros(1,4);
    cnt=1;
    k=1;
    %��ȡ�������ֵĲ���ֵ
    while k<length(rands)+1
        if abs(rands(k)-1)>0.01%���ʱ仯Ӧ�÷ǳ�С
            side_u(cnt)=u(k-1);
            cnt=cnt+1;
            while abs(rands(k)-1)>0.001
                k=k+1;
                if k==length(rands)
                    break;
                end
            end
            side_u(cnt)=u(k);
            cnt=cnt+1;
        end
        k=k+1;
    end
    DivisionCurve=DivisionFunction(nurbs(i),side_u(1),side_u(2),side_u(3));%�����
    side_nurbs{1,i}=cell2mat(DivisionCurve(1));%��Ϊ��ǰ��Ե�����ֵõ���˳������Ϊ��Ҷ������Ե��Ҷ�衢ǰԵ
    otherside_nurbs{1,i}=cell2mat(DivisionCurve(3));
end

