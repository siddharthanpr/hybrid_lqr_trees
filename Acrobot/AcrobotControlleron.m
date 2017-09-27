classdef AcrobotController < DrakeSystem
  properties
    p
  end
  methods
    function obj = AcrobotController(plant)
      obj = obj@DrakeSystem(0,0,4,1,true,true);
      obj.p = plant;
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,~,x)
        
          q = x(1:2);
      qd = x(3:4);
      
      % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
      q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
      q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));

      
      [f,df] = obj.p.dynamics(0,[pi;0;0;0],0);
      A = df(:,2:5);
      B = df(:,end);
      [K,S] = lqr(A,B,eye(4),1);
      xe = [q;qd] - [pi;0;0;0];
      
      if  0 %xe'*S*xe < 1000
        u = K*([pi;0;0;0] - [q;qd]);
      else
        % some useful functions + properties
        
        % acceleration due to gravity
        g = 9.81;
        m = obj.p.getMass;
        
        % computes center of mass [x;z] at any position
        com = obj.p.getCOM(q);
        
        % kinetic energy of a manipulator is 1/2 * qdot^T * H * qdot
        [H,C,B] = obj.p.manipulatorDynamics(q,qd);
        
        E_des = m*g*1.25;
        E_current = m*g*com(2) + .5*qd'*H*qd;
        
        M22bar = H(2,2) - H(2,1)/H(1,1)*H(1,2);
        h2bar = C(2) - H(2,1)/H(1,1)*C(1);
        k1 = 10;
        k2 = 10;
        k3 = 1;
        ubar = (E_des - E_current)*qd(2);

        u = M22bar*(-k1*q(2) - k2*qd(2)) + h2bar  + k3*ubar;
      end
      u = max(min(u,20),-20);
      x;
      % This is the end of the function
    end
  end
  
  methods (Static)
    function [t,x,x_grade]=run()
      plant = PlanarRigidBodyManipulator('Acrobot.urdf');
      controller = AcrobotController(plant);
      v = plant.constructVisualizer;
      sys_closedloop = feedback(plant,controller);
      
     %x0 = [.1*(rand(4,1) - 1)]; % start near the downward position
     x0 = [pi - pi*randn;0;0;0];  % start near the upright position
      xtraj=simulate(sys_closedloop,[0 15],x0);
      v.axis = [-4 4 -4 4];
      playback(v,xtraj);
      t = xtraj.pp.breaks;
      x = xtraj.eval(t);
      t_grade = linspace(3,4,98);
      x_grade = [xtraj.eval(0) xtraj.eval(t_grade) xtraj.eval(10)];
      x_grade = x_grade';
    end
  end
end
