clear all; clc;

N = [1, 2, 5, 10, 20, 40, 80, 120, 240, 320];
len = length(N);

h = zeros(1,len);
err = zeros(1,len);

for i=1:len
    n = N(i);
    h(i) = 1/n;
    err(i) = Demo(n+1);
end

loglog(h,err);
polyfit(log(h),log(err),1)
