classdef CompassGaitPlant < HybridDrakeSystem
  
  properties
    m = 5;
    mh = 10;
    a = 0.5;
    b = 0.5;
    g = 9.8;
    l;
    gamma = 3*pi/180;
    unom = 0;
  end
  
  methods 
      
%      function u = output(obj,t,~,x)
% %        if all(size(obj.unom)  == [1 1])
% %         u=0; 
% %         return; 
% %        end
%        
%        
%         if (t<obj.unom.traj{1}.tspan(2))
%         u= eval(obj.unom.traj{1},t);
%         else
%         u= eval(obj.unom.traj{2},t);
%         end
%       u = 0
%      end
     
     
    function obj = CompassGaitPlant()
      obj = obj@HybridDrakeSystem(1,4);
      obj.l=obj.a+obj.b;
      p = CompassGaitStancePlant(obj.m,obj.mh,obj.a,obj.b,obj.l,obj.g);
      obj = setInputFrame(obj,p.getInputFrame);
      obj = setOutputFrame(obj,p.getOutputFrame);
      
      obj = obj.addMode(p);
      obj = obj.addMode(p);

      obj = addTransition(obj,1,andGuards(obj,@footCollisionGuard1,@footCollisionGuard2),@collisionDynamics,false,true);
      obj = addTransition(obj,2,andGuards(obj,@footCollisionGuard1,@footCollisionGuard2),@collisionDynamics,false,true);

      %      obj.ode_options = odeset('InitialStep',1e-3, 'Refine',1,'MaxStep',0.02);
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    function [g,dg] = footCollisionGuard1(obj,t,x,u)
      g = x(1)+x(2)+2*obj.gamma;  % theta_st - gamma <= - (theta_sw - gamma) 
      dg = [0,1,1,0,0,0];
    end
    
    function [g,dg] = footCollisionGuard2(obj,t,x,u);
      g = -x(1);   % theta_sw >= 0
      dg = [0,-1,0,0,0,0];
    end
      
    function [xp,mode,status,dxp] = collisionDynamics(obj,mode,t,xm,u)
      m=obj.m; mh=obj.mh; a=obj.a; b=obj.b; l=obj.l;
      
      if (mode==1) mode=2;  % switch modes
      else mode=1; end
      
      alpha = (xm(1) - xm(2));
      Qm = [ -m*a*b, -m*a*b + (mh*l^2 + 2*m*a*l)*cos(alpha); 0, -m*a*b ];
      Qp = [ m*b*(b - l*cos(alpha)), m*l*(l-b*cos(alpha)) + m*a^2 + mh*l^2; m*b^2, -m*b*l*cos(alpha) ];
      
      xp = [xm([2,1]); ...     % switch stance and swing legs
        Qp\Qm*xm(3:4)];   % inelastic impulse
      
      if (nargout>3)
        Qpi = inv(Qp);
        dalpha = [1 -1];  % d/dq
        dQpdalpha = [m*b*l*sin(alpha), m*l*b*sin(alpha); 0, m*b*l*sin(alpha) ];
        dQpidalpha = -Qpi*dQpdalpha*Qpi;
        dQmdalpha = [0, -(mh*l^2 + 2*m*a*l)*sin(alpha); 0, 0];
        dxpdxm = zeros(4);
        dxpdxm(1:2,1:2) = [ 0 1 ; 1 0];
        dxpdxm(3:4,3:4) = Qpi*Qm;
        dxpdxm(3:4,1) = (dQpidalpha*dalpha(1))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(1))*xm(3:4);
        dxpdxm(3:4,2) = (dQpidalpha*dalpha(2))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(2))*xm(3:4);
        dxp = [zeros(4,2),dxpdxm,zeros(4,1)];
      end
      status = 0;
    end
    
  end
  
  methods (Static)
      

      function out = run()
        
      r = CompassGaitPlant();
      v = CompassGaitVisualizer(r);
      traj = simulate(r,[0 10]);%,[2;0; 0; 2.0; -0.4]);

      x0 = [0; 0; 2; -0.4];
      x0 =  [-0.32338855;0.21866879;-0.37718213;-1.0918269];
      xf = [0;0;0;0];
      x0_p = xf;
      epsilon = 1e-12;
%       while (norm(x0-x0_p)>epsilon)
%       x0_p=xf;
%       xf = strideFunction(r, x0);
%       x0=xf;
%       end
      xf1 = strideFunction(r, x0);


      eps = .00001;
      del_x = [eps;0;0;0];
      xf2 = strideFunction(r, x0+del_x);
      Adelx = xf2-xf1;
      A1= Adelx*pinv(del_x);
      
       del_x = [0;0;0;eps];
      xf2 = strideFunction(r, x0+del_x);
      Adelx = xf2-xf1;
      A4= Adelx*pinv(del_x);
     
      
       del_x = [0;eps;0;0];
      xf2 = strideFunction(r, x0+del_x);
      Adelx = xf2-xf1;
      A2= Adelx*pinv(del_x);
     
      
       del_x = [0;0;eps;0];
      xf2 = strideFunction(r, x0+del_x);
      Adelx = xf2-xf1;
      A3= Adelx*pinv(del_x);
      playback(v,traj);  
      A = A1 + A2 + A3 + A4;
      out = A;
   
    end
  end  
  
end
