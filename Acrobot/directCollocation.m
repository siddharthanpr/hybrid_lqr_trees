function [xtraj,utraj,z,F,info] = directCollocation( plant,x0,xf )

      N = 10;
      prog = DircolTrajectoryOptimization(plant,N,[0 5]);
      
      % add the initial value constraint
     
      prog = addStateConstraint(prog,BoundingBoxConstraint(x0,x0),1);
      
      % add the final value constraint
      
      prog = addStateConstraint(prog,BoundingBoxConstraint(xf,xf),N);
      
      % add the cost function g(dt,x,u) = 1*dt
      function [g,dg] = cost(dt,x,u)
         Q=diag([0.1 0.1 0.1 0.1]);
        R=1;
        xg = x-[pi;0;0;0];
        
        while xg(1)>pi
            xg(1) = xg(1)-pi;
        end
        while xg(1)<-pi 
              xg(1) = xg(1)+pi;
        end
        %
         g = dt+R*u^2 + xg'*Q*xg;
        dg = [1, 2*xg'*Q, 2*u*R];
          %g = dt; dg = [1,0*x',0*u']; % see geval.m for our gradient format
      end
      prog = addRunningCost(prog,@cost);
      
      % add a display function to draw the trajectory on every iteration
      function displayStateTrajectory(t,x,u)
          figure(1);
 
        plot(x(1,:),x(2,:),'b.-','MarkerSize',10);
        axis([-5,1,-2.5,2.5]); axis equal;
        drawnow;
      end
      prog = addTrajectoryDisplayFunction(prog,@displayStateTrajectory);
      
      % solve the optimization problem (with 2 sec as the initial guess for
      % the duration)
      prog = prog.setSolver('snopt');
      prog=prog.setSolverOptions('snopt','Algorithm','sqp');
     
     % global xtraj
     [xtraj,utraj,z,F,info] = prog.solveTraj(4);
end

