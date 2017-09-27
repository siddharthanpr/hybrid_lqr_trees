     function r = inregion(in)
            error1 = pi/3.8435;
             error2 = pi/1.8515;
            r = abs(  in(1))< (pi +error1) && in(1) > (pi -error1) && abs(in(2)) < (error2);
        end
      q = x(1:2);
      qd = x(3:4);
      
      % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
      q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
      q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));

      %%%% put your controler here %%%%
      % You might find some of the following functions useful
      % [H,C,B] = obj.p.manipulatorDynamics(q,qd);
      % com_position = obj.p.getCOM(q);
      % mass = obj.p.getMass();
      % gravity = obj.p.gravity;
      % Recall that the kinetic energy for a manpulator given by .5*qd'*H*qd
      u = 0;
      
       
      [H,C,B] = obj.p.manipulatorDynamics(q,qd);
      [f,df] = obj.p.dynamics(t,x,u);
   %
      persistent count
      persistent avA avB com_position K S x_init pinr pq px
      
      global sA sB
      
      
      
      
      com = obj.p.getCOM(q);
      
     
       E = .5*qd'*H*qd + com(2) * obj.p.getMass() * 9.8;

       com = obj.p.getCOM([pi,0]);
       Ed = com(2) * obj.p.getMass() * 9.8;
       
       a1=H(1,1);
       a2=H(1,2);
       a3=H(2,2);
       c1=C(1);
       c2=C(2);
       
       count;
        k1 = 1;
        k2 = 20;
        k3 = 2;
      
      
      
      qd(2);
       ue = - k1*(E-Ed)*qd(2);
       
       up= (-k2*q(2) -k3*qd(2) +c1*a2+c2*a3)/a3;
      u=ue+up;
        x_init = [pi 0 0 0]';
        if isempty(avA)
        avA = 0;
        avB = 0;
        count = 1;
        pinr=1;
        end
    
    
        count = count +1;
     
     
      
      if count >4 && count <=5
          avA = (avA*(count-5) + df(:,2:5))/(count-4);
      end
       if count >4 && count <=5
          avB = (avB*(count-5) + df(:,6))/(count-4);
       end
          Q = eye(4);
      R = 1;
       if count == 5
          avA = sA
          avB = sB
           
            [K,S] = lqr(avA,avB,Q,R)
       end
    q;   
    inr=inregion([q(1),q(2)]);
    if(pinr && ~inr)
    end
    pq=q;
    px=x;
    pinr = inr;
    if(count>4)
       
    %u=-K*([q;x(3);x(4)] - x_init);
    global rec
    if(q(1)<5)
    rec(count,:)=[q(1),q(2),(E-Ed)];
    end
    end
     if(count>4 && inregion([q(1),q(2)]) && (E-Ed) < 30 )
      [q' E-Ed]
      u=-K*([q;x(3);x(4)] - x_init);
     end
      %%%% end of your controller %%%%
   
  finalu = u;
      % leave this line below, it limits the control input to [-20,20]
      u = max(min(u,20),-20);
      x;
      % This is the end of the function
    end