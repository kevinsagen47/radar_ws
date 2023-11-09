%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
object1_cam_x=radar_cam_callib(:,1);
object1_radar_x=radar_cam_callib(:,3);
object1_y=radar_cam_callib(:,4);

object2_cam_x=radar_cam_callib(:,5);
object2_radar_x=radar_cam_callib(:,7);
object2_y=radar_cam_callib(:,8);

rmse_array=[];
for i=1:length(object1_y)
path1=GetPointLineDistance(x_bayes1(i),object1_y(i), ...
                            -1*route_p2(1,1),route_p2(2,1), ...
                            -1*route_p2(1,2),route_p2(2,2));

path2=GetPointLineDistance(x_bayes1(i),object1_y(i), ...
                            -1*route_p2(1,2),route_p2(2,2), ...
                            -1*route_p2(1,3),route_p2(2,3));

path3=GetPointLineDistance(x_bayes1(i),object1_y(i), ...
                            -1*route_p2(1,3),route_p2(2,3), ...
                            -1*route_p2(1,4),route_p2(2,4));

rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person1=mean(rmse_array)
min_person1=min(rmse_array)
max_person1=max(rmse_array)


rmse_array=[];
for i=1:length(object2_y)
path1=GetPointLineDistance(x_bayes2(i),object2_y(i), ...
                            -1*route_p1(1,1),route_p1(2,1), ...
                            -1*route_p1(1,2),route_p1(2,2));

path2=GetPointLineDistance(x_bayes2(i),object2_y(i), ...
                            -1*route_p1(1,2),route_p1(2,2), ...
                            -1*route_p1(1,3),route_p1(2,3));

path3=GetPointLineDistance(x_bayes2(i),object2_y(i), ...
                            -1*route_p1(1,3),route_p1(2,3), ...
                            -1*route_p1(1,4),route_p1(2,4));

rmse_array(end+1)=min([path1,path2,path3]);

end
RMSE_person2=mean(rmse_array)
min_person2=min(rmse_array)
max_person2=max(rmse_array)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





A=GetPointLineDistance(0.958,10.094,1.2,10.9,1.2,8.5)






function distance = GetPointLineDistance(x3,y3,x1,y1,x2,y2)
try
	
	% Find the numerator for our point-to-line distance formula.
	numerator = abs((x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1));
	
	% Find the denominator for our point-to-line distance formula.
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