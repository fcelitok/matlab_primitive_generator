clear;
close all;

wheelbase = 2.15;
steering_angle_max = 0.56; %0.61;
numberOfAngle = 32;
numberOfIntermediatePose = 15;
resolution = 0.25;
max_dist = 3;
linear_log = 'Linear';  %value could be linear or log
backward = 0;
backward_penalty = 3;

print_stuff = 0

min_radius = wheelbase/tan(steering_angle_max);

i = 2;
v = [0 1];
while (i * resolution <= max_dist)
    v = [v i];
    if (strcmp(linear_log,'linear'))
        i = i + 2;
    else
        i = i * 2;
    end
end
if (numberOfAngle >= 32)
    i = 3;
    while (i * resolution <= max_dist)
        v = [v i];
        if (strcmp(linear_log,'linear'))
            i = i + 3;
        else
            i = i * 3;
        end
    end   
end
if (numberOfAngle >= 64)
    i = 5;
    while (i * resolution <= max_dist)
        v = [v i];
        if (strcmp(linear_log,'linear'))
            i = i + 5;
        else
            i = i * 5;
        end
    end   
end

crowns = v(2:end)
v = [v v.*-1];
v = unique(v)

switch numberOfAngle
    case 16
        a=[0 1 2]
    case 32
        a=[0 1 2 3]
    case 64
        a=[0 1 2 3 5]
    otherwise
        a=[0 1 2]
end

angles = [];
a= unique([a a.*-1])
for i = a 
    for j = a
        if (j ~= 0 || i ~= 0)
            angles = [angles atan2(i,j)];
        end
    end
end
angles = unique(angles);
angles = [-angles(end) angles(1,1:end-1)]

if(print_stuff==1)
    figure();
    hold on;
    grid on;
    axis equal;
end

primID = 1;
for k = crowns
    for a1 = angles
        n = 0;
        for i = v 
            for j = v
                if (((abs(j)==k && abs(i)<=k) || (abs(i)==k && abs(j)<=k)))
                    for a2 = angles
                        a_start=a1;
                        a_end=a2;
                        d = sqrt((i * resolution)^2 + (j * resolution)^2 );
                        r_curv = abs(d/(2 * sin((a_end - a_start)/2)));
                        max_beta = 2*asin(d/(2*min_radius));
                        if     (a_start<a_end && a_start > -3.14 && ...
                                    r_curv >= min_radius && ...
                                    (a_end-a_start)<=max_beta && ...
                                    (a_start-atan2(j,i))<0 && ...
                                    (a_end-atan2(j,i))>0 && ...
                                    (a_end-a_start)>(a_start-atan2(j,i)) ...
                                    ) || ...
                                (a_start>a_end && a_start > -3.14 &&...
                                    r_curv >= min_radius && ...
                                    (a_start-a_end)<=max_beta && ...
                                    (a_start-atan2(j,i))>0 && ...
                                    (a_end-atan2(j,i))<0 && ...
                                    (a_end-a_start)<(a_start-atan2(j,i)) ...
                                    ) || ...
                                (a_start>a_end && a_start > -3.14 && a_end > -3.14 &&...
                                   r_curv >= min_radius && ...
                                   (a_end-a_start)<=-max_beta && ...
                                   (a_start-atan2(j,i))<0 && ...
                                   (a_end-atan2(j,i))>0 ...
                                  ) || ...
                                (a_start>a_end && a_start > -3.14 && a_end < -3.14 &&...
                                   r_curv >= min_radius && ...
                                   (a_end-a_start)<=-max_beta && ...
                                   (a_start-atan2(j,i))<0 && ...
                                   (-a_end-atan2(j,i))>0 ...
                                  ) || ...
                                (a_start<a_end && a_start <= -3.14 && ...
                                   r_curv >= min_radius && ...
                                  (a_end-a_start)>=max_beta && ...
                                  (-a_start-atan2(j,i))>0 && ...
                                  (a_end-atan2(j,i))<0 ...
                                 ) || ... 
                                (a_start<a_end && a_start <= -3.14 && ...
                                   r_curv >= min_radius && ...
                                  (a_end-a_start)<=max_beta && ...
                                  (a_start-atan2(j,i))<0 && ...
                                  (a_end-atan2(j,i))>0 ...
                                 )
                              
                                [r_curv min_radius a_end-a_start max_beta];
                                primitives(primID).start_pose = [0, 0, find(angles == a_start) - 1];
                                primitives(primID).end_pose = [i, j, find(angles == a_end) - 1];
                                primitives(primID).real_start_pose = [0 0 a_start];
                                primitives(primID).real_end_pose = [i * resolution, j * resolution, a_end];
                                [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = ...
                                     clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, ...
                                     numberOfIntermediatePose);
                                primID = primID + 1;
                                n = n + 1;
                                
                                if(print_stuff==1)
                                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                                    grid on, axis equal; hold on;
                                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                                        'Color',[0 0 0]);
                                    pause(0.1);
                                end                                
                            elseif(a_start==a_end && ((a_start==atan2(j,i) || (a_start+2*pi)==atan2(j,i))))
                                 primitives(primID).start_pose = [0, 0, find(angles == a_start) - 1];
                                 primitives(primID).end_pose = [i, j, find(angles == a_end) - 1];
                                 primitives(primID).real_start_pose = [0 0 a_start];
                                 primitives(primID).real_end_pose = [i * resolution, j * resolution, a_end];
                                 [primitives(primID).intermediate_poses, primitives(primID).additionalActionCostMult] = ...
                                     clotoide(primitives(primID).real_start_pose, primitives(primID).real_end_pose, ...
                                     numberOfIntermediatePose);
                                 primID = primID + 1;
                                 n = n + 1;
                                if(print_stuff==1)
                                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                                    grid on, axis equal; hold on;
                                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                                        'Color',[0 0 0]);
                                     pause(0.1);
                                end                                
                        end
                    end
                end
            end
        end
        if (n == 0)
            primitives(primID).start_pose = [0, 0, find(angles == a_start) - 1];
            primitives(primID).end_pose = [0, 0, find(angles == a_start) - 1];
            primitives(primID).additionalActionCostMult = 100;
            for p = 1:numberOfIntermediatePose
                primitives(primID).intermediate_poses(p,:) = [0, 0, a_start];
            end
            primID = primID + 1;
            if(print_stuff==1)
                plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                grid on, axis equal; hold on;
                quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            end                                
        end    
    end
end
primID-1
axis equal

generate_file(primitives, numberOfAngle, resolution);
%figure()
%for p = 1:(primID - 1)
%      plot(primitives(p).intermediate_poses(:,1),primitives(p).intermediate_poses(:,2),'+-b'), grid on, axis equal;
%      hold on, quiver(primitives(p).intermediate_poses(:,1),primitives(p).intermediate_poses(:,2),cos(primitives(p).intermediate_poses(:,3)),sin(primitives(p).intermediate_poses(:,3)),0.1,'Color',[0 0 0])
%end

