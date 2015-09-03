function [ flag ] = at_obstacle()
%Event which checks if we have detected any obstacle (within safe distance)
  
d_safe = 1000;
flag = 0;

  for i = 1:8
  
    r = arrobot_getsonarrange(i-1);
    if(r < d_safe)
        flag = 1;
        %disp('At obstacle event flaged');
        break
    end
    
  end
 
    
end

