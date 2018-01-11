function [ CheckingPoints ] = DecidePosition( CheckingPoints,VirtualPoints,Z_Value,Tolerance )
%DecidePosition 用于判断是否引入公差计算刀触点
%   毛坯曲面可能已经把设计曲面干掉了一部分（位于下公差带）
%   基于此，需要判断实际刀触点与毛坯曲线的位置关系
%   GrindingPoints此时为VirtualPoints在设计曲面上对应的最近点，Z_Value为相应的值，Tolerance为公差
n=size(CheckingPoints,2);%按列

%单独对最后进行判断
CheckingPoints(3,n)=Z_Value;
VirtualPoints(3,n)=Z_Value;
VirtualPoints(3,n-1)=Z_Value;
p_flag=JudgePosition(CheckingPoints(:,1),VirtualPoints(:,1),VirtualPoints(:,2));
if (p_flag>0)%在右边，右上或者右下
    %那么意味着该处的毛坯位于设计的下公差带
    %获取设计点指向毛坯点的矢量
    derive=VirtualPoints(:,1)-CheckingPoints(:,1);
    %沿该方向，对距离进行修改
    d=(Tolerance-norm(derive))/2;
    vector=derive/norm(derive);
    CheckingPoints(:,1)=VirtualPoints(:,1)+vector*d;%沿着该矢量方向加一个距离，保证在下公差
end

%无法进行并行计算
for i=1:1:n-1%对每个进行判断，判断方法与上面的相同
    CheckingPoints(3,i)=Z_Value;
    VirtualPoints(3,i)=Z_Value;
    VirtualPoints(3,i+1)=Z_Value;
    p_flag=JudgePosition(CheckingPoints(:,i),VirtualPoints(:,i),VirtualPoints(:,i+1));
    if (p_flag>0)%在右边，右上或者右下
        %那么意味着该处的毛坯位于设计的下公差带
        %获取设计点指向毛坯点的矢量
        derive=VirtualPoints(:,i)-CheckingPoints(:,i);
        %沿该方向，对距离进行修改
        d=(Tolerance-norm(derive))/2;
        vector=derive/norm(derive);
        CheckingPoints(:,i)=VirtualPoints(:,i)+vector*d;%沿着该矢量方向加一个距离，保证在下公差
    end
end

end

