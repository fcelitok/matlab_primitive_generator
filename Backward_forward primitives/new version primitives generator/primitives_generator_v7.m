clear all;
close all;

wheelbase = 0.2; % in meeters
steering_angle_max = 40*pi/180; % in radiants
numberOfAngle = 16; %16, 32, 48
numberOfIntermediatePose = 10;
resolution = 0.05;
max_dist = 2; %tested up tp 2.0m
linear_log = 'lin';  %value could be 'lin' or 'log'

forward = 1;
forward_penalty = 1;
backward = 1;
backward_penalty = 3;

% How many turns (left + right) maximum per each side
% reduce it to reduce the total number of primitives,
% althought this might reduce manouverability as well
brench_limit_angle = 2;
brench_limit_position = 2;

print_stuff=1;

min_radius = wheelbase/tan(steering_angle_max);
if (min_radius<max_dist)
    max_dist=min_radius;
end

i = 2;
v = [0 1];
while (i * resolution <= max_dist)
    v = [v i];
    if (strcmp(linear_log,'lin'))
        i = i + 2;
    else
        i = i * 2;
    end
end
if (numberOfAngle >= 32)
    i = 3;
    while (i * resolution <= max_dist)
        v = [v i];
        if (strcmp(linear_log,'lin'))
            i = i + 3;
        else
            i = i * 3;
        end
    end
end
if (numberOfAngle >= 48)
    i = 4;
    while (i * resolution <= max_dist)
        v = [v i];
        if (strcmp(linear_log,'lin'))
            i = i + 4;
        else
            i = i * 4;
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
    case 48
        a=[0 1 2 3 4]
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
angles = unique(angles)
angles = [-angles(end) angles(1,1:end-1)]
index = find(angles == 0)
angles = [angles(index:end) angles(1:index-1)]

if(print_stuff==1)
    figure();
    hold on;
    grid on;
    axis equal;
end

primID = 1;

if (forward == 1)
    for k = unique(crowns)
    y = [0:k-1 k*ones(1,2*k+1) k-1:-1:-k+1 -k*ones(1,2*k+1) -k+1:-1];
    x = [-k*ones(1,k) [-k:k-1] k*ones(1,2*k+1) k-1:-1:-k -k*ones(1,k-1)];
    %plot(x, y, 'k*')
    %pause(0.1)
    for a1=1:length(angles)
        %pause
        current_h=-1;
        for h=1:length(y)
            if(angles(a1) == atan2(y(h),x(h)) || angles(a1)+2*pi == atan2(y(h),x(h)))
                [a1 angles(a1)];
                plot(x(h)*resolution, y(h)*resolution, 'r*')
                current_h = h;
            end
        end
        if(current_h~=-1)
            for b=-min(k-1,brench_limit_position):min(k-1,brench_limit_position)
                if(current_h+b<=0)
                    select=length(y)+(current_h+b);
                elseif(current_h+b>length(y))
                    select=(current_h+b)-length(y);
                else
                    select=current_h+b;
                end
                for a=-min(k-1,brench_limit_angle):min(k-1,brench_limit_angle)
                    if(a1+a-b<=0)
                        a2=length(angles)+(a1+a-b);
                    elseif(a1+a-b>length(angles))
                        a2=(a1+a-b)-length(angles);
                    else
                        a2=a1+a-b;
                    end
                    a_start=angles(a1);
                    a_end=angles(a2);
                    d = sqrt((x(select) * resolution)^2 + (y(select) * resolution)^2 );
                    r_curv = abs(d/(2 * sin(angdiff(a_end,a_start)/2)));
                    max_beta = 2*asin(d/(2*min_radius));
                    [a_start a_end r_curv min_radius angdiff(a_end,a_start) max_beta];
                    
                    [clotoid_check, p, L ] = clotoide([0 0 a_start], [x(select) * resolution, y(select) * resolution, a_end], numberOfIntermediatePose, min_radius);
                    if (clotoid_check == true)
                        primitives(primID).start_pose = [0, 0, find(angles == a_start) - 1];
                        primitives(primID).end_pose = [x(select), y(select), find(angles == a_end) - 1];
                        primitives(primID).real_start_pose = [0 0 a_start];
                        primitives(primID).real_end_pose = [x(select) * resolution, y(select) * resolution, a_end];
                        primitives(primID).intermediate_poses = p;
                        primitives(primID).additionalActionCostMult = forward_penalty;
                        primID = primID + 1;

                        if(print_stuff==1)
                            plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                            grid on, axis equal; hold on;
                            quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                                'Color',[0 0 0]);
                            pause(0.1);
                        end
                    end %% TODO bu if e else koy bakalým 
                end
                plot(x(select)*resolution, y(select)*resolution, 'g*')
                pause(0.1)
            end
        else
            for b=-min(k-1,brench_limit_position):min(k-1,brench_limit_position)
                for a=-min(k-1,brench_limit_angle):min(k-1,brench_limit_angle)
                    primitives(primID).start_pose = [0, 0, find(angles == angles(a1))-1];
                    primitives(primID).end_pose = [0, 0, find(angles == angles(a1))-1];
                    primitives(primID).additionalActionCostMult = 100;
                    for p = 1:numberOfIntermediatePose
                        primitives(primID).intermediate_poses(p,:) = [0, 0, angles(a1)];
                    end
                    primID = primID + 1;
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
    primID-1
    end
end


if (backward == 1) 
    for k = unique(crowns)
        y = [0:k-1 k*ones(1,2*k+1) k-1:-1:-k+1 -k*ones(1,2*k+1) -k+1:-1];
        x = [-k*ones(1,k) [-k:k-1] k*ones(1,2*k+1) k-1:-1:-k -k*ones(1,k-1)];
        %plot(x, y, 'k*')
        %pause(0.1)
        for a1=1:length(angles)
            %pause
            current_h=-1;
            for h=1:length(y)
                if(angles(a1) == atan2(y(h),x(h)) + pi || angles(a1)+2*pi == atan2(y(h),x(h)) + pi || round(angles(a1),3) == round(atan2(y(h),x(h)) + 3.1416, 3))
                    [a1 angles(a1)];
                    plot(x(h)*resolution, y(h)*resolution, 'r*')
                    current_h = h;
                end
            end
            if(current_h~=-1)
                for b=-min(k-1,brench_limit_position):min(k-1,brench_limit_position)
                    if(current_h+b<=0)
                        select=length(y)+(current_h+b);
                    elseif(current_h+b>length(y))
                        select=(current_h+b)-length(y);
                    else
                        select=current_h+b;
                    end
                    for a=-min(k-1,brench_limit_angle):min(k-1,brench_limit_angle)
                        if(a1+a-b<=0)
                            a2=length(angles)+(a1+a-b);
                        elseif(a1+a-b>length(angles))
                            a2=(a1+a-b)-length(angles);
                        else
                            a2=a1+a-b;
                        end
                        a_start=angles(a1);
                        a_end=angles(a2);
                        d = sqrt((x(select) * resolution)^2 + (y(select) * resolution)^2 );
                        r_curv = abs(d/(2 * sin(angdiff(a_end,a_start)/2)));
                        max_beta = 2*asin(d/(2*min_radius));
                        [a_start a_end r_curv min_radius angdiff(a_end,a_start) max_beta];
                        [clotoid_check, p, L ] = clotoide([x(select) * resolution, y(select) * resolution, a_end], [0 0 a_start], numberOfIntermediatePose, min_radius);
                        % fix angle

                        if (clotoid_check == true)
                            primitives(primID).start_pose = [0, 0, find(angles == a_start) - 1];
                            primitives(primID).end_pose = [x(select), y(select), find(angles == a_end) - 1];
                            primitives(primID).real_start_pose = [0 0 a_start];
                            primitives(primID).real_end_pose = [x(select) * resolution, y(select) * resolution, a_end];
                            primitives(primID).intermediate_poses = flip(p);
                            primitives(primID).additionalActionCostMult = backward_penalty;
                            primID = primID + 1;

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
                    plot(x(select)*resolution, y(select)*resolution, 'g*')
                    pause(0.1)
                end
            else
                for b=-min(k-1,brench_limit_position):min(k-1,brench_limit_position)
                    for a=-min(k-1,brench_limit_angle):min(k-1,brench_limit_angle)
                        primitives(primID).start_pose = [0, 0, find(angles == angles(a1))-1];
                        primitives(primID).end_pose = [0, 0, find(angles == angles(a1))-1];
                        primitives(primID).additionalActionCostMult = 100;
                        for p = 1:numberOfIntermediatePose
                            primitives(primID).intermediate_poses(p,:) = [0, 0, angles(a1)];
                        end
                        primID = primID + 1;
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
        primID-1
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

