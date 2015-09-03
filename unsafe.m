function [ flag ] = unsafe()
%Event which checks if we have detected any obstacle (within unsafe distance)
  
d_unsafe = 500;
flag = 0;

  for i = 1:8
  
    r = arrobot_getsonarrange(i-1);
    if(r < d_unsafe)
        flag = 1;
        %disp('unsafe event flagged');
        break
    end
    
  end
 
  
end
