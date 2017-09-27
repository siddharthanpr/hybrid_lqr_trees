
clear tr
close all;
clc;


[p,utraj,xtraj,z,traj_opt]=runDircolCycle;
v = CompassGaitVisualizer(p);
xb = xtraj;
ub=utraj;
xtraj = xtraj.traj{1}.trajs{2}; % Extract first part of the trajectory (before collision)
utraj = utraj.traj{1};
ts = xtraj.getBreaks(); % Get time samples of trajectory
%utraj = PPTrajectory(spline(ts,zeros(1,length(ts)))); % Define nominal u(t),
                                                      % which is all zeros since our
                                                      % system was passive                                                      
                                                      
% Set frames
xtraj = xtraj.setOutputFrame(p.modes{1}.getStateFrame);
utraj = utraj.setOutputFrame(p.modes{1}.getInputFrame);


Q = diag([10 10 1 1]);
R = 1;
Qf = Q;

options = struct();

converged = false;
%Ad = findAd();
while ~converged
% tvlqr for continuous phase
[tv,V] = tvlqr(p.modes{1},xtraj,utraj,Q,R,Qf,options);
QfV = Qf;

% Jump equation (FILL ME IN) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S_t_plus = V.S.eval(0);
xend = xtraj.eval(ts(end));
[~,~,~,dxp] = p.collisionDynamics(1,0,xend,0); 
Ad = dxp(:,3:end-1);
S_t_minus = Ad'*S_t_plus*Ad;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Set Qf = to S_t_minus

Qf = S_t_minus;
[~,isp] = chol(Qf);



% Check for convergence (FILL ME IN)
if all(all(Qf - QfV)) < 1e-4
    converged = true;
end


end

Qf_converged = Qf;


[tv,V] = tvlqr(p.modes{1},xtraj,utraj,Q,R,Qf,options);

% Set frames of tvlqr controller
tv = tv.inOutputFrame(p.getInputFrame);
tv = tv.inInputFrame(p.getOutputFrame);

%pmodel = SimulinkModel(p.getModel());
pmodel = p.modes{1};
tv = tv.setInputFrame(pmodel.getOutputFrame);
tv = tv.setOutputFrame(pmodel.getInputFrame);
   
% Closed loop system (i.e., feedback system with TVLQR controller)
sysClosedLoop = feedback(pmodel,tv);

% Visualizer


%% Simulate from x0 (syntax is useful for part (b)) %%%%%%%%%%%%%%%%%%%%%%%
xtrajSim = sysClosedLoop.simulate([0 0.5], [xtraj.eval(0)]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 p_sys = taylorApprox(sysClosedLoop,xtrajSim,[],3);
Vparent=SpotPolynomialLyapunovFunction(V.getFrame(),V.getPoly(V.S.tspan(2)));
V=sampledFiniteTimeVerification(p_sys,xtrajSim.getBreaks(),Vparent,V,options);
plotFunnel(V.inFrame(p.getStateFrame),options); drawnow;
% Play trajectory back

xtrajSim = xtrajSim.setOutputFrame(p.getOutputFrame);
v.playback(xtrajSim);














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
