%ʹ�� ���ë���߽� ���ü� �����������
function pointlogicalValue = innerPtViaPolygon(segment, poly)
    %polyû�з�գ���һ���ȷ�ն����
    poly_close = [poly,poly(:,1)];
    % �����ж�poly������Щ��λ�ڶ���ε��ڲ�����߽���
    [in, on] = inpolygon(segment(1,:), segment(2,:),...
                         poly_close(1,:), poly_close(2,:));
    % ��ȡ�ڲ���λ�ã������ڲ��������ڱ߽���
    pointlogicalValue = in | on;
%     % �ü�poly�еĵ㣬��ֻ���ڲ��㱻����
%     pointlogicalValue = segment(:,ind); 
end

