function [ u ] = getTreeInput( x,t, u_flag )
global utraj;
%GETINPUT Summary of this function goes here
%   Detailed explanation goes here
% use LQRTree policy here to get inputs from LQR tree algorithm
        if u_flag
        
        ti = t;
        while(ti>utraj.traj{2}.tspan(2))
            ti = ti - utraj.traj{2}.tspan(2);
        end
        if (ti<utraj.traj{1}.tspan(2))
        u= eval(utraj.traj{1},ti);
        else
        u= eval(utraj.traj{2},ti);
        end
        
        else
            u=0;
        end
        
end

