%计算平底面与工件层相交线段中已切段、待切段的区域值
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

