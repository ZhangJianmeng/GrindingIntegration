% load datas.mat
%datas.mat将会提供一个五轴机床machine，两个刀位点Machinginput1和Machinginput2
%应用两个刀位点进行关键点等相关信息计算的类变量Machingpoisition
%MyWorkingMachine，即我们的工作机床，提供矩阵和矩阵求导的运算

[high_z,~]=Machingposition.GetHighest(10);%返回的是工件坐标系下的高度

%知道最高处后，只需要判断该处与最高层之间的关系即可，也许我们可以考虑到它的最高点问题，这个再议
z_theta=1;
%对每一层进行分析
%最好是返回每层的关键点的坐标，且在工件坐标系下
cnt=1;
key_points=cell(1,cnt);
for h=high_z+10:z_theta:high_z+20
    key_points{1,cnt}=Machingposition.CalKeyPoints(h,10);%划分5次
    cnt=cnt+1;
end
%
% indx=[];
% a=zeros(1,1);
% for i=1:1:length(key_points)
%     for j=1:1:6
%         if isempty(key_points{1,i}{1,j})
%             indx=[indx,[i;j]];
%         end
%     end
% end

%把两个工位点画出来
[ points ] = PlotStartEnd(high_z+10,Machingposition );
n=size(points,2);
for i=1:1:n
    plot3(points(1,i),points(2,i),points(3,i),'*');
    hold on;
end

for i=1:1:length(key_points)
    points=cell2mat(key_points{1,i});
    n=size(points,2);
    for j=1:1:n%按列的个数来取
        plot3(points(1,j),points(2,j),points(3,j),'*');
        hold on;
    end
end