%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
for i=1:length(scene_1_cam_radar_fusion(:,1))
path1=GetPointLineDistance(scene_1_cam_radar_fusion(i,7),scene_1_cam_radar_fusion(i,8), ...
                            -1*route3_p2(1,1),route3_p2(2,1), ...
                            -1*route3_p2(1,2),route3_p2(2,2));

path2=GetPointLineDistance(scene_1_cam_radar_fusion(i,7),scene_1_cam_radar_fusion(i,8), ...
                            -1*route3_p2(1,2),route3_p2(2,2), ...
                            -1*route3_p2(1,3),route3_p2(2,3));

path3=GetPointLineDistance(scene_1_cam_radar_fusion(i,7),scene_1_cam_radar_fusion(i,8), ...
                            -1*route3_p2(1,3),route3_p2(2,3), ...
                            -1*route3_p2(1,4),route3_p2(2,4));

rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person1_scene1=rmse_array;
radar_RMSE_person1_scene1=rms(rmse_array)


rmse_array=[];
for i=1:length(scene_1_cam_radar_fusion(:,6))
path1=GetPointLineDistance(scene_1_cam_radar_fusion(i,2),scene_1_cam_radar_fusion(i,3), ...
                            -1*route3_p1(1,1),route3_p1(2,1), ...
                            -1*route3_p1(1,2),route3_p1(2,2));

path2=GetPointLineDistance(scene_1_cam_radar_fusion(i,2),scene_1_cam_radar_fusion(i,3), ...
                            -1*route3_p1(1,2),route3_p1(2,2), ...
                            -1*route3_p1(1,3),route3_p1(2,3));

path3=GetPointLineDistance(scene_1_cam_radar_fusion(i,2),scene_1_cam_radar_fusion(i,3), ...
                            -1*route3_p1(1,3),route3_p1(2,3), ...
                            -1*route3_p1(1,4),route3_p1(2,4));

rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person2_scene1=rmse_array;
radar_RMSE_person2_scene1=rms(rmse_array)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
for i=1:length(scene_2_cam_radar_fusion(:,1))
path1=GetPointLineDistance(scene_2_cam_radar_fusion(i,7),scene_2_cam_radar_fusion(i,8), ...
                            -1*route2_p1(1,1),route2_p1(2,1), ...
                            -1*route2_p1(1,2),route2_p1(2,2));

path2=GetPointLineDistance(scene_2_cam_radar_fusion(i,7),scene_2_cam_radar_fusion(i,8), ...
                            -1*route2_p1(1,2),route2_p1(2,2), ...
                            -1*route2_p1(1,3),route2_p1(2,3));

path3=GetPointLineDistance(scene_2_cam_radar_fusion(i,7),scene_2_cam_radar_fusion(i,8), ...
                            -1*route2_p1(1,3),route2_p1(2,3), ...
                            -1*route2_p1(1,4),route2_p1(2,4));


rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person1_scene2=rmse_array;
radar_RMSE_person1_scene2=rms(rmse_array)


rmse_array=[];
for i=1:length(scene_2_cam_radar_fusion(:,6))
path1=GetPointLineDistance(scene_2_cam_radar_fusion(i,2),scene_2_cam_radar_fusion(i,3), ...
                            -1*route2_p2(1,1),route2_p2(2,1), ...
                            -1*route2_p2(1,2),route2_p2(2,2));

path2=GetPointLineDistance(scene_2_cam_radar_fusion(i,2),scene_2_cam_radar_fusion(i,3), ...
                            -1*route2_p2(1,2),route2_p2(2,2), ...
                            -1*route2_p2(1,3),route2_p2(2,3));

path3=GetPointLineDistance(scene_2_cam_radar_fusion(i,2),scene_2_cam_radar_fusion(i,3), ...
                            -1*route2_p2(1,3),route2_p2(2,3), ...
                            -1*route2_p2(1,4),route2_p2(2,4));

rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person2_scene2=rmse_array;
radar_RMSE_person2_scene2=rms(rmse_array)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmse_array=[];
for i=1:length(scene_3_cam_radar_fusion(:,1))
path1=GetPointLineDistance(scene_3_cam_radar_fusion(i,7),scene_3_cam_radar_fusion(i,8), ...
                            -1*route1_p2(1,1),route1_p2(2,1), ...
                            -1*route1_p2(1,2),route1_p2(2,2));

path2=GetPointLineDistance(scene_3_cam_radar_fusion(i,7),scene_3_cam_radar_fusion(i,8), ...
                            -1*route1_p2(1,2),route1_p2(2,2), ...
                            -1*route1_p2(1,3),route1_p2(2,3));

path3=GetPointLineDistance(scene_3_cam_radar_fusion(i,7),scene_3_cam_radar_fusion(i,8), ...
                            -1*route1_p2(1,3),route1_p2(2,3), ...
                            -1*route1_p2(1,4),route1_p2(2,4));
path4=GetPointLineDistance(scene_3_cam_radar_fusion(i,7),scene_3_cam_radar_fusion(i,8), ...
                            -1*route1_p2(1,4),route1_p2(2,4), ...
                            -1*route1_p2(1,5),route1_p2(2,5));

rmse_array(end+1)=min([path1,path2,path3,path4]);

end
RMSE_person1_scene3=rmse_array;
radar_RMSE_person1_scene3=rms(rmse_array)


rmse_array=[];
for i=1:length(scene_3_cam_radar_fusion(:,6))
path1=GetPointLineDistance(scene_3_cam_radar_fusion(i,2),scene_3_cam_radar_fusion(i,3), ...
                            -1*route1_p1(1,1),route1_p1(2,1), ...
                            -1*route1_p1(1,2),route1_p1(2,2));

path2=GetPointLineDistance(scene_3_cam_radar_fusion(i,2),scene_3_cam_radar_fusion(i,3), ...
                            -1*route1_p1(1,2),route1_p1(2,2), ...
                            -1*route1_p1(1,3),route1_p1(2,3));

path3=GetPointLineDistance(scene_3_cam_radar_fusion(i,2),scene_3_cam_radar_fusion(i,3), ...
                            -1*route1_p1(1,3),route1_p1(2,3), ...
                            -1*route1_p1(1,4),route1_p1(2,4));
path4=GetPointLineDistance(scene_3_cam_radar_fusion(i,2),scene_3_cam_radar_fusion(i,3), ...
                            -1*route1_p1(1,4),route1_p1(2,4), ...
                            -1*route1_p1(1,5),route1_p1(2,5));

rmse_array(end+1)=min([path1,path2,path3,path4]);

end
RMSE_person2_scene3=rmse_array;
radar_RMSE_person2_scene3=rms(rmse_array)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%A=GetPointLineDistance(0.958,10.094,1.2,10.9,1.2,8.5)

mean([mean_RMSE_person1_scene1, ...
    mean_RMSE_person2_scene1, ...
    mean_RMSE_person1_scene2, ...
    mean_RMSE_person2_scene2, ...
    mean_RMSE_person1_scene3, ...
    mean_RMSE_person2_scene3])




function distance = GetPointLineDistance(x3,y3,x1,y1,x2,y2)
try
	
	% Find the numerator for our point-to-line distance formula.
	numerator = abs((x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1));
    %numerator = abs((x2 - x1) * (x1 - x3));
	
	% Find the denominator for our point-to-line distance formula.
	%denominator = abs( (x2 - x1));
    denominator = sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2);
	
	% Compute the distance.
	distance = numerator ./ denominator;
catch ME
	callStackString = GetCallStack(ME);
	errorMessage = sprintf('Error in program %s.\nTraceback (most recent at top):\n%s\nError Message:\n%s',...
		mfilename, callStackString, ME.message);
	WarnUser(errorMessage)
end
return; % from GetPointLineDistance()
end