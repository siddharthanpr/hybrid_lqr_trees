clear all
clc
close all
load limCycle
query1 = [   0.174431646531995
  -0.000236367718194
   2.095029812346471
  -0.171602277479299];

% query1 = [     -1.000000000000000
%    0.300000000000000
%    2.095029812346471
%   -0.171602277479299];

% query1 = [     1.500000000000000
%    0.300000000000000
%    2.095029812346471
%   -0.171602277479299];


ts = xtraj.getBreaks();
Veval = [];
for i = 1:length(ts)
    Veval(i) = V.eval(ts(i),query1);
    
end

[Vmin,imin] = min(Veval);

[p,utraj1,xtraj1,z,traj_opt] = runDircolCycleInputs(query1,eval(xtraj,ts(imin)));

v = CompassGaitVisualizer(p);

temp = xtraj.shiftTime(xtraj1.tspan(end));
xtrajTrim = temp.trim([ts(imin)+xtraj1.tspan(end), temp.tspan(end)]);
xtrajTrim = xtrajTrim.setOutputFrame(p.getOutputFrame);
T = xtrajTrim.tspan(end);
temp = xtraj.shiftTime(T);

for i = 1:3
T = temp.tspan(end);
temp = append(temp,xtraj.shiftTime(T));
end

temp = temp.setOutputFrame(p.getOutputFrame);
v.playback(xtraj1);
v.playback(xtrajTrim);
v.playback(temp);
close all;
figure (1);
hold on
dt = 0.0001;


for t = 0:dt:xtraj.tspan(end)
    xt = eval(xtraj,t);
    drawCircle(.045,[xt(1) xt(3)], [.5 .5 .5] );
    drawCircle(.045,[xt(2) xt(4)], [.5 .5 .5] );
    
end
for t = 0:dt:xtraj1.tspan(end)
    xt = eval(xtraj1,t);
    drawCircle(.025,[xt(2) xt(4)], [.75 .75 .75] );
    drawCircle(.025,[xt(3) xt(5)], [.75 .75 .75] );
    
end

fnplt(xtraj1,[3 5])
fnplt(xtraj1,[2 4])
fnplt(xtraj,[2 4])
fnplt(xtraj,[1 3])
xlabel('q');
ylabel('q_{dot}');
figure (2);
fnplt(utraj1);