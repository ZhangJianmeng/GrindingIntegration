clear all;clc;close all;
% // Define four NURBS curves and construct a Coons surface patch.
pnts1 = [ 0.0  3.0  4.5  6.5 8.0 10.0;
    0.0  0.0  0.0  0.0 0.0  0.0;
    0  0  0  0 0  0];
A=[0 0 0 1/3 0.5 2/3 1 1 1];
% U=optknt(A,2+1);          %获得最优节点划分
crv1 = nrbmak(pnts1,A);

pnts2= [ 0.0  3.0  5.0  8.0 10.0;
    10.0 10.0 10.0 10.0 10.0;
    0  0  0  0 0];
crv2 = nrbmak(pnts2, [0 0 0 1/3 2/3 1 1 1]);

pnts3= [ 0.0 -1.0 0.0 0.0;
    0.0 3.0 8.0 10.0;
    0 0.0 0 0];
crv3 = nrbmak(pnts3, [0 0 0 0.5 1 1 1]);

pnts4= [ 10.0 10.0 10.0 10.0 10.0;
    0.0   3.0  5.0  8.0 10.0;
    0   0  0 0 0];
crv4 = nrbmak(pnts4, [0 0 0 0.25 0.75 1 1 1]);

srf1 = nrbcoons(crv1, crv2, crv3, crv4);
%   nrbplot(srf,[20 20],220,45);
hold on;
nrbctrlplot(crv1);
nrbctrlplot(crv2);
nrbctrlplot(crv3);
nrbctrlplot(crv4);
nrbctrlplot(srf1);
hold on;

pnts11=[ 0.0  3.0  4.5  6.5 8.0 10.0;
    0.0  0.0  0.0  0.0 0.0  0.0;
    0  0  0  0 0  0];
pnts22=[ 0.0  3.0  5.0  8.0 10.0;
    10.0 3.0 10.0 10.0 10.0;
    0  0  0  0 0];
pnts33=[ 10.0 10.0 10.0 10.0 10.0;
    0.0   3.0  5.0  8.0 10.0;
    0   0  0 0 0];
pnts44=[ 0.0 0 0.0 0.0;
    0.0 3.0 8.0 10.0;
    0 0.0 0 0];


pnts11(1,:)=pnts1(1,:)+10;
crv1 = nrbmak(pnts11, [0 0 0 1/3 0.5 2/3 1 1 1]);

pnts22(1,:)=pnts2(1,:)+10;
crv2 = nrbmak(pnts22, [0 0 0 1/3 2/3 1 1 1]);

pnts33(1,:)=pnts33(1,:);
crv3 = nrbmak(pnts33, [0 0 0 0.25 0.75 1 1 1]);

pnts44(1,:)=pnts3(1,:)+20;
crv4 = nrbmak(pnts44, [0 0 0 0.5 1 1 1]);

nrbctrlplot(crv1);
nrbctrlplot(crv2);
nrbctrlplot(crv3);
nrbctrlplot(crv4);

srf2 = nrbcoons(crv1, crv2, crv3, crv4);
nrbctrlplot(srf2);



% 获得节点矢量搜在区间序号和改节点处的基函数

