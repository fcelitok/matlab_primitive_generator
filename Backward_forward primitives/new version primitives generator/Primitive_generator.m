clear;
close all;

matrix_8_fw = [-1  0 0;
               -1 -1 1;
                0 -1 2;
                1 -1 3;
                1  0 4;
                1  1 5;
                0  1 6;
               -1  1 7];
           
matrix_8_bw = [ 1  0 0;
                1  1 1;
                0  1 2;
               -1  1 3;  
               -1  0 4;
               -1 -1 5;
                0 -1 6;
                1 -1 7];
            
matrix_16_fw = [-2  0 0;
                -2 -1 1;
                -2 -2 2;
                -1 -2 3;
                 0 -2 4;
                 1 -2 5;
                 2 -2 6;
                 2 -1 7;
                 2  0 8;
                 2  1 9;
                 2  2 10;
                 1  2 11;
                 0  2 12;
                -1  2 13;
                -2  2 14;
                -2  1 15];
            
matrix_16_bw = [ 2  0 0;
                 2  1 1;
                 2  2 2;
                 1  2 3;
                 0  2 4;
                -1  2 5;
                -2  2 6;
                -2  1 7;
                -2  0 8;
                -2 -1 9;
                -2 -2 10;
                -1 -2 11;
                 0 -2 12;
                 1 -2 13;
                 2 -2 14;
                 2 -1 15];
            
matrix_32_fw = [-8  0 0;
                -8 -4 1;
                -8 -8 2;
                -4 -8 3;
                 0 -8 4;
                 4 -8 5;
                 8 -8 6;
                 8 -4 7;
                 8  0 8;
                 8  4 9;
                 8  8 10;
                 4  8 11;
                 0  8 12;
                -4  8 13;
                -8  8 14;
                -8  4 15];  
            
matrix_32_bw = [ 8  0 0;
                 8  4 1;
                 8  8 2;
                 4  8 3;
                 0  8 4;
                -4  8 5;
                -8  8 6;
                -8  4 7; 
                -8  0 8;
                -8 -4 9;
                -8 -8 10;
                -4 -8 11;
                 0 -8 12;
                 4 -8 13;
                 8 -8 14;
                 8 -4 15];
                           
matrix_32_fw_plus = [-8 -2 1;
                     -8 -6 2;
                     -6 -8 3;
                     -2 -8 4;
                      2 -8 5;
                      6 -8 6;
                      8 -6 7;
                      8 -2 8;
                      8  2 9;
                      8  6 10;
                      6  8 11;
                      2  8 12;
                     -2  8 13;
                     -6  8 14;
                     -8  6 15;
                     -8  2 0];
                 
matrix_32_bw_plus = [ 8  2 1;
                      8  6 2;
                      6  8 3;
                      2  8 4;
                     -2  8 5;
                     -6  8 6;
                     -8  6 7;
                     -8  2 8;
                     -8 -2 9;
                     -8 -6 10;
                     -6 -8 11;
                     -2 -8 12;
                      2 -8 13;
                      6 -8 14;
                      8 -6 15;
                      8 -2 0];
                 
matrix_32_fw_minus = [-8  2 15;
                      -8 -2 0;
                      -8 -6 1;
                      -6 -8 2;
                      -2 -8 3;
                       2 -8 4;
                       6 -8 5;
                       8 -6 6;
                       8 -2 7;
                       8  2 8;
                       8  6 9;
                       6  8 10;
                       2  8 11;
                      -2  8 12;
                      -6  8 13;
                      -8  6 14];
                 
matrix_32_bw_minus = [ 8 -2 15;
                       8  2 0;
                       8  6 1;
                       6  8 2;
                       2  8 3;
                      -2  8 4;
                      -6  8 5;
                      -8  6 6;
                      -8  2 7;
                      -8 -2 8;
                      -8 -6 9;
                      -6 -8 10;
                      -2 -8 11;
                       2 -8 12;
                       6 -8 13;
                       8 -6 14];
                  
resolution = 0.25;        
number_of_angle = 16;
angles = [0, atan2(1,2), atan2(2,2), atan2(2,1), pi/2, atan2(2,-1), atan2(2,-2), atan2(1,-2), -pi, atan2(-1,-2), atan2(-2,-2), atan2(-2,-1), -pi/2, atan2(-2,1), atan2(-2,2), atan2(-1,2)];
angles = sort(angles);
numberOfIntermediatePose = 15;
bw_penalty = 3;

plot_primitives = 0;
if (plot_primitives)
    figure()
end
primID = 1;
angles_8_index = 1:2:16;


for i = 1:size(matrix_8_fw,1)
    primitives(primID).start_pose = [0 0 angles_8_index(i)-1];
    primitives(primID).end_pose = [matrix_8_fw(i,1), matrix_8_fw(i,2), angles_8_index(i)-1];
    primitives(primID).real_start_pose = [0 0 angles(angles_8_index(i))];
    primitives(primID).real_end_pose = [matrix_8_fw(i,1) * resolution, matrix_8_fw(i,2) * resolution, angles(angles_8_index(i))];
    [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);    
    primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * 1.2;
    if (plot_primitives)
        plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
        hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
    end
    primID = primID + 1;
    
%     primitives(primID).start_pose = [0 0 angles_8_index(i)-1];
%     primitives(primID).end_pose = [matrix_8_bw(i,1), matrix_8_bw(i,2), angles_8_index(i)-1];
%     primitives(primID).real_start_pose = [0, 0, angles(angles_8_index(i)) + pi];
%     primitives(primID).real_end_pose = [matrix_8_bw(i,1) * resolution, matrix_8_bw(i,2) * resolution, angles(angles_8_index(i)) + pi];
%     [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);  
%     primitives(primID).intermediate_poses(:,3) =  primitives(primID).intermediate_poses(:,3) - pi;
%     primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * 1.2 * bw_penalty;
%     if (plot_primitives)
%         plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
%         hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
%     end
%     primID = primID + 1;
end

for i = 2:2:16
    primitives(primID).start_pose = [0 0 i-1];
    primitives(primID).end_pose = [0, 0, i-1];
    primitives(primID).additionalActionCostMult = 1000;
    for j = 1:numberOfIntermediatePose
        primitives(primID).intermediate_poses(j,:) = [0 0 angles(i)];
    end
    primID = primID + 1;
    
%     primitives(primID).start_pose = [0 0 i-1];
%     primitives(primID).end_pose = [0, 0, i-1];
%     primitives(primID).additionalActionCostMult = 1000;
%     for j = 1:numberOfIntermediatePose
%         primitives(primID).intermediate_poses(j,:) = [0 0 angles(i)];
%     end    
%     primID = primID + 1;
end

for i = 1:size(matrix_16_fw,1)
    primitives(primID).start_pose = [0 0 i-1];
    primitives(primID).end_pose = [matrix_16_fw(i,1), matrix_16_fw(i,2), i-1];
    primitives(primID).real_start_pose = [0 0 angles(i)];
    primitives(primID).real_end_pose = [matrix_16_fw(i,1) * resolution, matrix_16_fw(i,2) * resolution, angles(i)];
    [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);  
    primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * 1.1;
    if (plot_primitives)
        plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
        hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
    end
    primID = primID + 1;
   
%     primitives(primID).start_pose = [0 0 i-1];
%     primitives(primID).end_pose = [matrix_16_bw(i,1), matrix_16_bw(i,2), i-1];
%     primitives(primID).real_start_pose = [0 0 angles(i) + pi];
%     primitives(primID).real_end_pose = [matrix_16_bw(i,1) * resolution, matrix_16_bw(i,2) * resolution, angles(i) + pi];
%     [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);  
%     primitives(primID).intermediate_poses(:,3) =  primitives(primID).intermediate_poses(:,3) - pi;
%     primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * 1.1 * bw_penalty;
%     if (plot_primitives)
%         plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
%         hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
%     end
%     primID = primID + 1;
end

for i = 1:size(matrix_32_fw,1)
    primitives(primID).start_pose = [0 0 i-1];
    primitives(primID).end_pose = [matrix_32_fw(i,1), matrix_32_fw(i,2), i-1];
    primitives(primID).real_start_pose = [0 0 angles(i)];
    primitives(primID).real_end_pose = [matrix_32_fw(i,1) * resolution, matrix_32_fw(i,2) * resolution, angles(i)];
    [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);     
    if (plot_primitives)
        plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
        hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
    end
    primID = primID + 1;
    
%     primitives(primID).start_pose = [0 0 i-1];
%     primitives(primID).end_pose = [matrix_32_bw(i,1), matrix_32_bw(i,2), i-1];
%     primitives(primID).real_start_pose = [0 0 angles(i) + pi];
%     primitives(primID).real_end_pose = [matrix_32_bw(i,1) * resolution, matrix_32_bw(i,2) * resolution, angles(i) + pi];
%     [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);
%     primitives(primID).intermediate_poses(:,3) =  primitives(primID).intermediate_poses(:,3) - pi;
%     primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * bw_penalty;
%     if (plot_primitives)
%         plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
%         hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])    
%     end
%     primID = primID + 1;
end


for i = 1:size(matrix_32_fw_plus,1)
    primitives(primID).start_pose = [0 0 i-1];
    primitives(primID).real_start_pose = [0 0 angles(i)];
    if (i == 16)
        primitives(primID).end_pose = [matrix_32_fw_plus(i,1), matrix_32_fw_plus(i,2), 0];
        primitives(primID).real_end_pose = [matrix_32_fw_plus(i,1) * resolution, matrix_32_fw_plus(i,2) * resolution, angles(1)];
    else
        primitives(primID).end_pose = [matrix_32_fw_plus(i,1), matrix_32_fw_plus(i,2), i];
        primitives(primID).real_end_pose = [matrix_32_fw_plus(i,1) * resolution, matrix_32_fw_plus(i,2) * resolution, angles(i+1)];
    end
    [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);      
    if (plot_primitives)
        plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
        hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
    end
    primID = primID + 1;
    
%     primitives(primID).start_pose = [0 0 i-1];
%     primitives(primID).real_start_pose = [0 0 angles(i) + pi];
%     if (i == 16)
%         primitives(primID).end_pose = [matrix_32_bw_plus(i,1), matrix_32_bw_plus(i,2), 0];
%         primitives(primID).real_end_pose = [matrix_32_bw_plus(i,1) * resolution, matrix_32_bw_plus(i,2) * resolution, angles(1) + pi];
%     else
%         primitives(primID).end_pose = [matrix_32_bw_plus(i,1), matrix_32_bw_plus(i,2), i];
%         primitives(primID).real_end_pose = [matrix_32_bw_plus(i,1) * resolution, matrix_32_bw_plus(i,2) * resolution, angles(i+1) + pi];
%     end
%     [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);      
%     primitives(primID).intermediate_poses(:,3) =  primitives(primID).intermediate_poses(:,3) - pi;
%     primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * bw_penalty;
%     if (plot_primitives)
%             plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
%             hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
%     end
%     primID = primID + 1;
end

for i = 1:size(matrix_32_fw_minus,1)
    primitives(primID).start_pose = [0 0 i-1];
    primitives(primID).real_start_pose = [0 0 angles(i)];
    if (i == 1)
         primitives(primID).end_pose = [matrix_32_fw_minus(i,1), matrix_32_fw_minus(i,2), 15];
         primitives(primID).real_end_pose = [matrix_32_fw_minus(i,1) * resolution, matrix_32_fw_minus(i,2) * resolution, angles(16)];
    else
         primitives(primID).end_pose = [matrix_32_fw_minus(i,1), matrix_32_fw_minus(i,2), i-2];
         primitives(primID).real_end_pose = [matrix_32_fw_minus(i,1) * resolution, matrix_32_fw_minus(i,2) * resolution, angles(i-1)];
    end
    [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);     
    if (plot_primitives)
        plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
        hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
    end
    primID = primID + 1;
    
%     primitives(primID).start_pose = [0 0 i-1];
%     primitives(primID).real_start_pose = [0 0 angles(i) + pi];
%     if (i == 1)
%          primitives(primID).end_pose = [matrix_32_bw_minus(i,1), matrix_32_bw_minus(i,2), 15];
%          primitives(primID).real_end_pose = [matrix_32_bw_minus(i,1) * resolution, matrix_32_bw_minus(i,2) * resolution, angles(16) + pi];
%     else
%          primitives(primID).end_pose = [matrix_32_bw_minus(i,1), matrix_32_bw_minus(i,2), i-2];
%          primitives(primID).real_end_pose = [matrix_32_bw_minus(i,1) * resolution, matrix_32_bw_minus(i,2) * resolution, angles(i-1) + pi];
%     end
%     [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, numberOfIntermediatePose);     
%     primitives(primID).intermediate_poses(:,3) =  primitives(primID).intermediate_poses(:,3) - pi;
%     primitives(primID).additionalActionCostMult = primitives(primID).additionalActionCostMult * bw_penalty;
%     if (plot_primitives)
%         plot(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),'+-b'), grid, axis equal;
%         hold on, quiver(primitives(primID).intermediate_poses(:,1),primitives(primID).intermediate_poses(:,2),cos(primitives(primID).intermediate_poses(:,3)),sin(primitives(primID).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
%     end
%     primID = primID + 1;
end


generate_file(primitives, number_of_angle, resolution);




