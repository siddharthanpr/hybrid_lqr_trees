function [] = drawCircle( r,pos,color )
%DRAWCIRCLE Summary of this function goes here
%   Detailed explanation goes here
t = 0:.1:2*pi;
h = fill(pos(1)+r*cos(t),pos(2)+2*r*sin(t),color);
set(h,'EdgeColor','None');
end

