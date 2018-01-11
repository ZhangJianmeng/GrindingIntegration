%����ƽ�����빤�����ཻ�߶������жΡ����жε�����ֵ
function [cuttingDom,cutDom] = calcLineDomain(key,cond)

    if ~isnan(key)
        if cond
            cuttingDom = [-1,key];
            cutDom = [key,1];
        else
            cuttingDom = [key,1];
            cutDom = [-1,key];
        end
    else
        if cond
            cuttingDom = [-1,1];
            cutDom = NaN;
        else
            cuttingDom = NaN;
            cutDom = [-1,1];
        end
    end

end

