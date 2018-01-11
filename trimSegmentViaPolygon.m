%ʹ�� ��ն���� ���ü� �߶�
function validSegment = trimSegmentViaPolygon(segment, poly)
    %polyû�з�գ���һ���ȷ�ն����
    poly_close = [poly,poly(:,1)];
    % �����ж�poly������Щ��λ�ڶ���ε��ڲ�����߽���
    [in, on] = inpolygon(segment(1,:), segment(2,:),...
                         poly_close(1,:), poly_close(2,:));
    % ��ȡ�ⲿ��λ�ã��������ڲ��������ڱ߽���
    ind = ~in | on;
    % �ü�poly�еĵ㣬��ֻ���ⲿ�㱻����
    validSegment = segment(:,ind); 
end

