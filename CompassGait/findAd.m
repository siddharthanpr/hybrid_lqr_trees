 function out = findAd()
        
      r = CompassGaitPlant();
      r.gamma=0;
      v = CompassGaitVisualizer(r);
     % traj = simulate(r,[0 10]);%,[2;0; 0; 2.0; -0.4]);
      x0 = [0; 0; 2; -0.4];
      x0 =  [   0;0.021804910166189;0.863919287063885;-0.141701250442245];
     xf = strideFunction(r, x0); 
     
      epsilon = 1e-3;
      while (norm(x0-xf)>epsilon)
          
          x0=xf;
      xf = strideFunction(r, x0);   
      
      end
      xf1 = strideFunction(r, x0);


      eps = .001;
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
      %playback(v,traj);  
      A = A1 + A2 + A3 + A4;
      out = A;
   
    end