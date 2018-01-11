function [ New_Nurbs ] = Getsection(Total_Surface,Height,Height_para,n)
% �ú�������ȷ��������Zƽ�洦������
% HeightΪ��ʵ���������е�Zֵ��Height_para�ǽ����������ĳ�ʼֵ��������0��1����
% nΪȡ������֮ǰΪ2000�������޸�Ϊ�������
% Height��ʾ�߶ȣ���zֵ
k=3;%��������
u=linspace(0,1,n);
U_Vparameter=cell(1,length(Height));
New_Nurbs=cell(1,length(Height));
for i=1:length(Height)
    x0=[u;ones(1,length(u))*Height_para(i)];
    [True_v(:,:)]=Newton_section(Height(i),x0,Total_Surface);%ţ�ٵ��������Ӧzֵ����u,v��
    U_Vparameter{1,i}=(True_v);
end
for i=1:length(Height)
    [p,~]=nrbeval(Total_Surface,U_Vparameter{1,i});
    New_Nurbs{1,i}=GetNurbsSpline(p,k);%�õ�����׼����B��������
end
end

