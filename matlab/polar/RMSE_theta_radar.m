%RMSE of 3 scenarios fusion 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
length_1=97;
fused_theta1=all_scene_1(1:length_1,2);
fused_theta2=all_scene_1(1:length_1,13);
theta_gt1=theta2_gt_p1_s1;
theta_gt2=theta2_gt_p2_s1;
for path_set=1:(length(theta_gt1(1,:))-1)
    for frame_x=theta_gt1(1,path_set):(theta_gt1(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta1(frame_x), ...
            theta_gt1(1,path_set),theta_gt1(2,path_set), ...
            theta_gt1(1,path_set+1),theta_gt1(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_radar_p1_s1=rmse_array;
RMSE_theta_radar_p1_s1=rms(rmse_array)

rmse_array=[];
for path_set=1:(length(theta_gt2(1,:))-1)
    for frame_x=theta_gt2(1,path_set):(theta_gt2(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta2(frame_x), ...
            theta_gt2(1,path_set),theta_gt2(2,path_set), ...
            theta_gt2(1,path_set+1),theta_gt2(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_fused_p2_s1=rmse_array;
RMSE_theta_radar_p2_s1=rms(rmse_array)
%{
f=figure(1);
clf(f,"reset")
plot(error_theta_fused_p1_s1,'LineWidth',2);
hold on;
plot(error_theta_fused_p2_s1,'LineWidth',2);
hold off;
xlabel('Frame (#)')
ylabel('Error (rad)')
ylim([0 0.35])
lgd=legend('Object 1','Object 2','Location','Best');
title('Fused Scenario 1 Azimuth Error')
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
length_1=95;
fused_theta1=all_scene_2(1:length_1,2);
fused_theta2=all_scene_2(1:length_1,13);
theta_gt2=theta2_gt_p1_s2;
theta_gt1=theta2_gt_p2_s2;
for path_set=1:(length(theta_gt1(1,:))-1)
    for frame_x=theta_gt1(1,path_set):(theta_gt1(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta1(frame_x), ...
            theta_gt1(1,path_set),theta_gt1(2,path_set), ...
            theta_gt1(1,path_set+1),theta_gt1(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_fused_p1_s2=rmse_array;
RMSE_theta_radar_p1_s2=rms(rmse_array)

rmse_array=[];
for path_set=1:(length(theta_gt2(1,:))-1)
    for frame_x=theta_gt2(1,path_set):(theta_gt2(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta2(frame_x), ...
            theta_gt2(1,path_set),theta_gt2(2,path_set), ...
            theta_gt2(1,path_set+1),theta_gt2(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_fused_p2_s2=rmse_array;
RMSE_theta_radar_p2_s2=rms(rmse_array)
%{
f=figure(2);
clf(f,"reset")
plot(error_theta_fused_p1_s2,'LineWidth',2);
hold on;
plot(error_theta_fused_p2_s2,'LineWidth',2);
hold off;
xlabel('Frame (#)')
ylabel('Error (rad)')
ylim([0 0.35])
lgd=legend('Object 1','Object 2','Location','Best');
title('Fused Scenario 2 Azimuth Error')
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
length_1=113;
fused_theta1=all_scene_3(16:length_1,2);
fused_theta2=all_scene_3(16:length_1,13);
theta_gt1=theta2_gt_p1_s3;
theta_gt2=theta2_gt_p2_s3;
for path_set=1:(length(theta_gt1(1,:))-1)
    for frame_x=theta_gt1(1,path_set):(theta_gt1(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta1(frame_x), ...
            theta_gt1(1,path_set),theta_gt1(2,path_set), ...
            theta_gt1(1,path_set+1),theta_gt1(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_fused_p1_s3=rmse_array;
RMSE_theta_radar_p1_s3=rms(rmse_array)

rmse_array=[];
for path_set=1:(length(theta_gt2(1,:))-1)
    for frame_x=theta_gt2(1,path_set):(theta_gt2(1,path_set+1)-1)
        error=get_point_2_gt_dist(frame_x,fused_theta2(frame_x), ...
            theta_gt2(1,path_set),theta_gt2(2,path_set), ...
            theta_gt2(1,path_set+1),theta_gt2(2,path_set+1));
        rmse_array(end+1)=error;
    end
end

%error_theta_fused_p2_s3=rmse_array;
RMSE_theta_radar_p2_s3=rms(rmse_array)
%{
f=figure(3);
clf(f,"reset")
plot(error_theta_fused_p1_s3,'LineWidth',2);
hold on;
plot(error_theta_fused_p2_s3,'LineWidth',2);
hold off;
xlabel('Frame (#)')
ylabel('Error (rad)')
ylim([0 0.35])
lgd=legend('Object 1','Object 2','Location','Best');
title('Fused Scenario 3 Azimuth Error')
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function theta_distance = get_point_2_gt_dist(x3,y3,x1,y1,x2,y2)
%x3 and y3 is the point
%x1 y1 and x2 y2 is the line aka ground truth
try
    coefficients = polyfit([x1, x2], [y1, y2], 1);
    a = coefficients (1);
    b = coefficients (2);

    %y_1=@(x)a*x+b;
	
	% Compute the distance.
	theta_distance = abs(y3-(a*x3+b));
catch ME
	callStackString = GetCallStack(ME);
	errorMessage = sprintf('Error in program %s.\nTraceback (most recent at top):\n%s\nError Message:\n%s',...
		mfilename, callStackString, ME.message);
	WarnUser(errorMessage)
end
return; % from GetPointLineDistance()
end