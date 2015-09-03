function [ flag ] = obstacle_cleared()
%Event which checks if we have caleared all obstacle (within safe distance)
  
d_safe = 1000;

flag = 1;

  for i = 1:8
  
    r = arrobot_getsonarrange(i-1);
    if(r < d_safe)
        flag = 0;
    end
    
  end
 
 if(flag == 1)
     %disp('obstacle cleared event flagged');
 end
 
end

