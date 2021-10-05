function [xCenter, yCenter, radius, meanError, maxError] = FitCircle(x, y)
% FitCircle(): Fits a circle through a set of points in the x - y plane.
% USAGE :
% [xCenter, yCenter, radius, a] = FitCircle(X, Y)
% The output is the center point (xCenter, yCenter) and the radius of the fitted circle.
% "a" is an optional output vector describing the coefficients in the circle's equation:
%     x ^ 2 + y ^ 2 + a(1) * x + a(2) * y + a(3) = 0
% by Bucher Izhak 25 - Oct - 1991

numPoints = numel(x);
xx = x .* x;
yy = y .* y;
xy = x .* y;
A = [sum(x),  sum(y),  numPoints;
     sum(xy), sum(yy), sum(y);
     sum(xx), sum(xy), sum(x)];
B = [-sum(xx + yy) ;
     -sum(xx .* y + yy .* y);
     -sum(xx .* x + xy .* y)];
a = A \ B;
xCenter = -.5 * a(1);
yCenter = -.5 * a(2);
radius  =  sqrt((a(1) ^ 2 + a(2) ^ 2) / 4 - a(3));

errors = sqrt(abs(x.^2 + y.^2 + a(1) * x + a(2) * y + a(3)));

meanError = mean(errors);
maxError = max(errors);
