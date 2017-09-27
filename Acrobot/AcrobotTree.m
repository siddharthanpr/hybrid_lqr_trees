% Acrobot model
clear tr
p = PlanarRigidBodyManipulator('Acrobot.urdf');

% Fixed point (upright configuration)
x0 = Point(p.getStateFrame,[pi;0;0;0]);
u0 = Point(p.getInputFrame,zeros(1,1));

% LQR controller
Q = diag([10;10;1;1]);
R = 0.1;
[c,V0] = tilqr(p,x0,u0,Q,R);
tr = LQRTree(x0,u0,c,V0);
options.Tslb = 0;
options.Tsub = 5;
%  x0 = randomSample;%[0.1*(rand(4,1) - 1)]; % start near the downward position
%  %     x0 = [pi - .1*randn;0;0;0];  % start near the upright position
%  xf = [pi;0;0;0];
% % 
% % 
%  [xtraj,utraj,z,F,info] = directCollocation1(p,x0,xf,4,false);




c = LQRTree.buildLQRTree(p,x0,u0,@randomSample,Q,R,options);

% 
%  x0 = [pi-.051;0;0;0];%[0.1*(rand(4,1) - 1)]; % start near the downward position
%  %     x0 = [pi - .1*randn;0;0;0];  % start near the upright position
%  xf = [pi;0;0;0];
% % 
% % 
%  [xtraj,utraj,z,F,info] = directCollocation1(p,x0,xf,4,false);
% 
% [c,V0] = tvlqr(p,xtraj,utraj,Q,R,V0);
% tr.addTrajectory(xtraj,utraj,c,V0);
% tr.checkFunnels([1;1;1;1]);
% %tr.buildLQRTree(p,x0,u0,@randomSample,Q,R);
