function [] = generate_file(primitives, numberofangles, resolution_m)

totalnumberofprimitives = size(primitives,2);

filename = 'v8_forward_backward_25deg_ang1_pos2_reduced_check.mprim';
fileID = fopen(filename,'w');
fprintf(fileID, 'resolution_m: %1.6f\n', resolution_m);
fprintf(fileID, 'numberofangles: %d\n', numberofangles);
fprintf(fileID, 'totalnumberofprimitives: %d\n', totalnumberofprimitives);

% Primitives writing
for i = 1:size(primitives,2)
    primitive = primitives(i);
    fprintf(fileID, 'primID: %d\n', i-1);
        
        
        states = primitive.intermediate_poses;
        %additionalcost = primitive.objective;
        %if(primitive.nlpinfo>3 || primitive.nlpinfo<0)
	    %   additionalcost = additionalcost *100;
        %end
        intermediateposes = size(states,1);
        
        fprintf(fileID, 'startangle_c: %d\n', primitive.start_pose(3));
        fprintf(fileID, 'endpose_c: %d %d %d\n', primitive.end_pose);
        fprintf(fileID, 'additionalactioncostmult: %d\n', primitive.additionalActionCostMult);
        fprintf(fileID, 'intermediateposes: %d\n', intermediateposes);
        fprintf(fileID, '%2.4f %2.4f %2.4f\n', states');
end
fclose(fileID);

