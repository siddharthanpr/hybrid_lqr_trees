% x = linspace(-10,10,1000);
% xdot = x.^3 + 2*x.^2 - 5*x - 6;
% 
% eq_points = [-3;-1;2];
% figure(1)
% hold
% plot(x,xdot);
% plot(x,zeros(max(size(xdot))),'r');     \\\


syms x
x_star = 0;
h = 1.9;
f = -x;
xvect(1)=0.01;
for i=2:200
xvect(i) = xvect(i-1) + h*-xvect(i-1); 
end      
xvect(200)