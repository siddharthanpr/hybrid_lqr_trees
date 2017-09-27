
close all;
clc;
p = CompassGaitPlant();
p.gamma = 0;
v= CompassGaitVisualizer(p);
% load unom;
% global utraj;
% utraj = ut;
 xtraj = simulate(p,[0 20],[1;ddc]);
 
 %newbest 1.000000000000000                   0   0.030659746556237   0.916659290371034  -0.134417045757548
 
 
 %best [   1.000000000000000;0;0.021804910166189;0.863919287063885;-0.141701250442245]
 v.playback(xtraj);
 %about to fall: [ 1.000000000000000;0;0.02704910166189;1.063919287063885;-0.471701250442245]
 %falls over[ 1.000000000000000;0;0.02704910166189;1.63919287063885;-0.471701250442245]
figure;
hold on;
fnplt(xtraj, [1 3]);
fnplt(xtraj, [2 4]);