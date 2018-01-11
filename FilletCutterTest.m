%% 面向对象测试
clear;
tic
%初始化: 机床,刀具,工件,刀位点 对象
    machine = FiveAxisMachine(0, 0, 77.5);
    cutter = FilletCutter(5,0,20);
    CL1 = CutterLocation(0,0,-2,0,0.2873,0.9574);
    CL2 = CutterLocation(3,-1,-1,0.57735,0.57735,0.57735);
    wB=[0,25,25,0;-15,-15,15,15];   % 存储毛坯边界坐标值
    zTemp=-3.5;   
    workpiece = Workpiece(-13.123, 3.631, 42.5,zTemp,wB,-6); 
    T1=0.82;
    T2=1;
    workingMach = WorkingMachine(machine,cutter,workpiece,CL1,CL2);
    [z0max1,z0min1] = workingMach.clcWorkpieceLayerDomain(T1);
     z0minT1 = workingMach.clcWorkpieceLayerDomainAll(T1);
     z0minT2 = workingMach.clcWorkpieceLayerDomainAll(T2);
     
     
    %画离散时刻瞬时切削刃
    figure
    for t = 0:0.05:1
        workingMach.showSection(t);
        hold on
    end
    %画包络线
    figure
    workingMach.plotSmoothBoundary(1,zTemp);
    %画包络线上的点
    figure
    Pt=workingMach.findSmoothBoundary(1,zTemp);
    scatter(Pt(1,:),Pt(2,:),5,'o','MarkerEdgeColor',[0,0.7,0.9]);

    z0theta=0.3; 
    
%画包络线三维
figure
  for z0=(z0minT1+0.005):z0theta:(z0minT1+3)
%   for z0=-2.5:z0theta:0
       workpiece = Workpiece(-13.123, 3.631, 42.5, z0,wB,-6);     
    %启动一台机床
       workingMach = WorkingMachine(machine,cutter,workpiece,CL1,CL2);
% 测试
       workingMach.plotSmoothBoundary(T1,z0);
       hold on
%        workingMach.plotWorkpieceBoundary(z0);
%        hold on
  end
  
%  figure
%  for z0=(z0minT1+0.005):z0theta:0
% %   for z0=-2.5:z0theta:0
%        workpiece = Workpiece(-13.123, 3.631, 42.5, z0,wB,-6);     
%     %启动一台机床
%        workingMach = WorkingMachine(machine,cutter,workpiece,CL1,CL2);
%        
%     % 测试
%        workingMach.builtActualTimeWorkpieceBoundary(T1,z0);
%        hold on
%        xlabel('x');
%        ylabel('y');
%        zlabel('z');
%   end
 
    
  


  toc