% RUN THIS to generate your solution
megaclear

[p,xtraj,utraj,v,x0] = pset5_catch;

% if you want to display the trajectory again
%v.playback(xtraj);

% ********YOUR CODE HERE ********
% Set Q, R, and Qf for time varying LQR
% See problem statement for instructions here
  xf = xtraj.eval(3);
  q = xf(1:5);
  qd = xf(6:10);
  kinsol = p.doKinematics(q);
  
  % body index, so p.body(3) is the lower link
  hand_body = 3;
  
  % position of the "hand" on the lower link, 2.1m is the length
  pos_on_hand_body = [0;-2.1];
  
  % Calculate position of the hand in world coordinates
  % the gradient, dHand_pos, is the derivative w.r.t. q
  [hand_pos,dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
  dHand_pos;

d1 = -[dHand_pos(1,1);dHand_pos(2,1)];

d2 = -[dHand_pos(1,2);dHand_pos(2,2)];

d3  = [1;0];
d4  = [0;1];


f = [q(3);q(4)] - hand_pos;
   ball_pos = [q(3);q(4)];
dBall_pos = [zeros(2,2) eye(2) zeros(2,1)];

 df = [0 0 1 0 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0 0] - [dHand_pos(1,:) 0 0 0 0 0;dHand_pos(2,:) 0 0 0 0 0];
 %df= ones(2,10);
 
  
 
Qf=[2*(d1')*d1 2*d1'*d2  2*d1'*d3 2*d1'*d4 zeros(1,6); 2*d2'*d1 2*(d2')*d2 2*d2'*d3 2*d2'*d4 zeros(1,6); 2*d3'*d1 2*d3'*d2 2*(d3')*d3 2*d3'*d4 zeros(1,6);2*d4'*d1 2*d4'*d2 2*d4'*d3 2*(d4')*d4 zeros(1,6);zeros(6,10)];












R = 1;

Q =eye(10);
Qf=Qf/2;

Qf-df'*df
Qf=Qf*100;
Qf = Qf+Q;

% The Hessian is actually df'*df!

% *******************************

c = p.tvlqr(xtraj,utraj,Q,R,Qf);
sys_cl = p.feedback(c);

%%
x0_test = x0;
x0_test(3) = x0(3) + .1;
traj_test_1 = sys_cl.simulate(xtraj.tspan,x0_test);
v.playback(traj_test_1);
xf1 = traj_test_1.eval(traj_test_1.tspan(2))
x0_test = x0 + .02*(rand(10,1) - 1);
traj_test_2 = sys_cl.simulate(xtraj.tspan,x0_test);
v.playback(traj_test_2);
xf2 = traj_test_2.eval(traj_test_2.tspan(2))
% submit x_grade below
x_grade = [traj_test_1.eval(xtraj.pp.breaks) traj_test_2.eval(xtraj.pp.breaks) Qf repmat(xtraj.tspan(2),10,1)]';

format short
%display(x_grade)
format long