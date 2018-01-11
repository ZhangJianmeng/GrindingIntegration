function Curve = ControlCurve( ControlPoints,order )
%�õ����ƶ����γɵ�����
%������ƶ�������õ��Ŀ����ߵĴ���(order)
%�����������
%% ���ýڵ�ʸ��
n=size(ControlPoints,2);
delt_u=1/(n-order);
mid=(0:delt_u:1);
start=zeros(1,order);
knot_end=ones(1,order);
knot_u=[start,mid,knot_end];
%% �õ�����
Curve=nrbmak(ControlPoints,knot_u);
end
