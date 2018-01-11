function Curve = ControlCurve( ControlPoints,order )
%得到控制顶点形成的曲线
%输入控制顶点和欲得到的控制线的次数(order)
%输出控制曲线
%% 配置节点矢量
n=size(ControlPoints,2);
delt_u=1/(n-order);
mid=(0:delt_u:1);
start=zeros(1,order);
knot_end=ones(1,order);
knot_u=[start,mid,knot_end];
%% 得到曲线
Curve=nrbmak(ControlPoints,knot_u);
end
