%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%
length_1=97;
cam_theta1  =all_scene_1(1:length_1,1);
radar_theta1=all_scene_1(1:length_1,2);
fused_theta1=all_scene_1(1:length_1,3);

cam_theta2  =all_scene_1(1:length_1,12);
radar_theta2=all_scene_1(1:length_1,13);
fused_theta2=all_scene_1(1:length_1,14);

theta_x_axis=uint32([1:length(cam_theta1)]);
clf
f = figure(1);
f.Position = [100 100 550*1.4 400*1.4];
hold on
scatter(theta_x_axis,cam_theta1,"filled")
scatter(theta_x_axis,radar_theta1,"filled")
scatter(theta_x_axis,fused_theta1,"filled")
plot([1,64,89,97],-1*theta_gt_p1_s1,'red')

scatter(theta_x_axis,cam_theta2,"filled")
scatter(theta_x_axis,radar_theta2,"filled")
scatter(theta_x_axis,fused_theta2,"filled")
plot([1,29,52,95],-1*theta_gt_p2_s1,'blue')
lgd=legend('Camera Object 1','Radar Object 1','Fused Object 1',"Ground Truth Object 1",'Camera Object 2','Radar Object 2','Fused Object 2',"Ground Truth Object 2",'Location','Best');
xlabel('Frame (#)')
ylabel('Azimuth (rad)')
title('Radar vs Camera Azimuth Angle Scenario 1')
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
cam_theta1  =theta_scene_2(1:95,1);
cam_theta2  =theta_scene_2(1:95,3);
radar_theta1=theta_scene_2(1:95,2);
radar_theta2=theta_scene_2(1:95,4);
theta_x_axis=uint32([1:length(cam_theta1)]);
clf
f = figure(1);
f.Position = [100 100 550*1.4 400*1.4];
scatter(theta_x_axis,theta_scene_2(1:95,1),"filled")
hold on
%CAMERA
scatter(theta_x_axis,theta_scene_2(1:95,2),"filled")
scatter(theta_x_axis,theta_scene_2(1:95,3),"filled")
scatter(theta_x_axis,theta_scene_2(1:95,4),"filled")
plot([1,28,66,95],-1*theta_gt_p1_s2,'red')
plot([1,25,64,95],-1*theta_gt_p2_s2,'blue')
lgd=legend('Camera Object 1','Radar Object 1','Camera Object 2','Radar Object 2',"Ground Truth Object 1","Ground Truth Object 2",'Location','Best');
xlabel('Frame (#)')
ylabel('Azimuth (rad)')
title('Radar vs Camera Azimuth Angle Scenario 3')
hold off

%}

%BAYES
%scatter(object_fusion_x1,object_fusion_y1+5.5,"filled")

%plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',2)