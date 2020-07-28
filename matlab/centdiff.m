function difference = centdiff(x, t)
%CENTDIFF Summary of this function goes here
%   Detailed explanation goes here
    difference = zeros(size(x));
    for i = 2:size(x,1)-1
        difference(i, :) = (x(i+1, :)-x(i-1, :))/(t(i+1)-t(i-1));
    end
end

