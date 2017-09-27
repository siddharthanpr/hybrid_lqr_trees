function [ s ] = add( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
persistent k 

if isempty(k)
  k = 1;
end
k = k+1

 


end

