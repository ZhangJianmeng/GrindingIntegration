%使用 封闭多边形 来裁剪 线段
function validSegment = trimSegmentViaPolygon(segment, poly)
    %poly没有封闭，第一步先封闭多边形
    poly_close = [poly,poly(:,1)];
    % 首先判断poly上有哪些点位于多边形的内部，或边界上
    [in, on] = inpolygon(segment(1,:), segment(2,:),...
                         poly_close(1,:), poly_close(2,:));
    % 获取外部点位置：即不在内部，或者在边界上
    ind = ~in | on;
    % 裁剪poly中的点，即只有外部点被保留
    validSegment = segment(:,ind); 
end

