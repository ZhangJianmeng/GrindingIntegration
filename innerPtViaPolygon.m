%使用 封闭毛坯边界 来裁剪 工件层包络线
function pointlogicalValue = innerPtViaPolygon(segment, poly)
    %poly没有封闭，第一步先封闭多边形
    poly_close = [poly,poly(:,1)];
    % 首先判断poly上有哪些点位于多边形的内部，或边界上
    [in, on] = inpolygon(segment(1,:), segment(2,:),...
                         poly_close(1,:), poly_close(2,:));
    % 获取内部点位置：即在内部，或者在边界上
    pointlogicalValue = in | on;
%     % 裁剪poly中的点，即只有内部点被保留
%     pointlogicalValue = segment(:,ind); 
end

