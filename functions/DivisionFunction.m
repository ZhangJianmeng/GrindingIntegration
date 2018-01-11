function [DivisionCurve]=DivisionFunction(OriginCurve,U1,U2,U3)
%-----------OriginCurve表示要分割的曲线
%-----------u表示在u处分割曲线
%% 曲线的分割函数编写
% for i=1:length(OriginCurve)
    U=[U1,U2,U3];
    K=3; % 曲线的次数
    %% 分割第一段曲线
    KnotsIndex=findspan (length(OriginCurve.coefs)-1, K, U,OriginCurve.knots);
    b=OriginCurve.knots(1:KnotsIndex(1)+1);
    points=nrbeval(OriginCurve,U);
    NewKnots1=[b U(1)*ones(1,K+1)];
    c=OriginCurve.coefs(:,1:KnotsIndex(1)-K+1);
    inurbs = nrbkntins(OriginCurve,[U(1)*ones(1,K),U(2)*ones(1,K),U(3)*ones(1,K)]);
    % d1=setdiff(inurbs.coefs',OriginCurve.coefs','rows');
    % [m,n]=intersect(inurbs.coefs',d1,'rows');
    [index0,index00]=intersect(roundn(inurbs.coefs(1:3,:)',-8),roundn(points(:,1)',-8),'rows');
    NewCoefs1=inurbs.coefs(:,1:index00);
    for j=1:length(NewKnots1)
        NewKnots1(j)=(NewKnots1(j)-NewKnots1(1))/(NewKnots1(end)-NewKnots1(1));
    end
%     nrbctrlplot(inurbs);
    DivisionCurve{1,1}=nrbmak(NewCoefs1,NewKnots1);
%     nrbctrlplot(DivisionCurve{1,1});
%     hold on;
    %% 第二段曲线表达式的求解
    % [index0,index00]=intersect(roundn(inurbs.coefs(1:3,:)',-8),roundn(points(:,1)',-8),'rows');
    [index1,index2]=intersect(roundn(inurbs.coefs(1:3,:)',-8),roundn(points(:,2)',-8),'rows');
    NewKnots2=[U(1)*ones(1,K+1),OriginCurve.knots(KnotsIndex(1)+2:KnotsIndex(2)+1),U(2)*ones(1,K+1)];
    NewCoefs2=inurbs.coefs(:,index00:index2);
    DivisionCurve{1,2}=nrbmak(NewCoefs2,NewKnots2);
%     nrbctrlplot(DivisionCurve{1,2});
    %% 第三段曲线表达式的求解
    [index3,index33]=intersect(roundn(inurbs.coefs(1:3,:)',-8),roundn(points(:,3)',-8),'rows');
    NewKnots3=[U(2)*ones(1,K+1),OriginCurve.knots(KnotsIndex(2)+2:KnotsIndex(3)+1),U(3)*ones(1,K+1)];
    NewCoefs3=inurbs.coefs(:,index2:index33);
    DivisionCurve{1,3}=nrbmak(NewCoefs3,NewKnots3);
%   nrbctrlplot(DivisionCurve{1,3});
%     hold on;
%     u11=linspace(0,1,50);
%     pointsa=nrbeval(DivisionCurve{i,3},u11);
%     plot3(pointsa(1,1:17),pointsa(2,1:17),pointsa(3,1:17));
%     hold on;
%     plot3(pointsa(1,17:36),pointsa(2,17:36),pointsa(3,17:36));
%     plot3(pointsa(1,36:end),pointsa(2,36:end),pointsa(3,36:end));
%     set(h2(i),'color','k','linewidth',2);
%     %% 第四段曲线表达式的求解
%     [index4,index44]=intersect(roundn(inurbs.coefs(1:3,:)',-8),roundn(points(:,4)',-8),'rows');
%     NewKnots4=[U(3)*ones(1,K+1),OriginCurve.knots(KnotsIndex(3)+2:KnotsIndex(4)+1),U(4)*ones(1,K+1)];
%     NewCoefs4=inurbs.coefs(:,index33:index44);
%     DivisionCurve{1,4}=nrbmak(NewCoefs4,NewKnots4);
%     nrbctrlplot(DivisionCurve{1,4});
    %% 最后一段曲线
    NewKnots5=[U(3)*ones(1,K+1) OriginCurve.knots(KnotsIndex(3)+K-1:end) ];
    NewCoefs5=inurbs.coefs(:,index33:end);
    DivisionCurve{1,4}=nrbmak(NewCoefs5,NewKnots5);
%     nrbctrlplot(DivisionCurve{1,4});
% end
end