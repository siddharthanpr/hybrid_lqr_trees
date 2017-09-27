 function [t,xf] = timeStrideFunction(p, x0)
         temp = p.gamma;
         p.gamma = 3*pi/180;
         
         Htraj = simulate(p,[0 1], [1;x0]);
         t = Htraj.traj{2}.tspan(1);
         xf = Htraj.traj{2}.eval(t);
         p.gamma = temp;
        
   end