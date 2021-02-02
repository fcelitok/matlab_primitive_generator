function [interpolatePoses] = linear_interpolation(start_pose,end_pose,numberOfIntermediatePose)
    x_interp =  linspace(start_pose(1),end_pose(1),numberOfIntermediatePose);
    x = [start_pose(1), end_pose(1)];
    y = [start_pose(2), end_pose(2)];
    if (start_pose(1) ~= 0 | end_pose(1) ~= 0)      
        y_interp = interp1(x, y, x_interp);
    else 
        y_interp = linspace(start_pose(2),end_pose(2) ,numberOfIntermediatePose);  
    end
    angle_interp = linspace(start_pose(3),end_pose(3) ,numberOfIntermediatePose);
    
    interpolatePoses = [x_interp' y_interp', angle_interp'];