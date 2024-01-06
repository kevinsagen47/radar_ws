%figure 1 plot raw camera data and radar data
%figure 2 plot fused result
clf

object1_cam_x=all_scene_1(:,6);
object1_y=all_scene_1(:,7);

%object1_radar_x=scene_1_cam_radar_fusion(:,2);
object1_radar_x=all_scene_1(:,10);
object1_radar_y=all_scene_1(:,11);

object_fusion_x1=all_scene_1(:,6);
object_fusion_y1=all_scene_1(:,7);

object2_cam_x=all_scene_1(:,17);
object2_y=all_scene_1(:,18);

%object2_radar_x=scene_1_cam_radar_fusion(:,7);
object2_radar_x=all_scene_1(:,21);
object2_radar_y=all_scene_1(:,22);

object_fusion_x2=all_scene_1(:,17);
object_fusion_y2=all_scene_1(:,18);

cam_cov1=cov(object1_cam_x);
radar_cov1=cov(object1_radar_x);
%x_bayes1=((object1_radar_x/radar_cov1^2)+(object1_cam_x/cam_cov1^2))/((1/radar_cov1^2)+(1/cam_cov1^2));


cam_cov2=cov(object2_cam_x);
radar_cov2=cov(object2_radar_x);
%x_bayes2=((object2_radar_x/radar_cov2^2)+(object2_cam_x/cam_cov2^2))/((1/radar_cov2^2)+(1/cam_cov2^2));

f = figure(1);
f.Position = [100 100 550*1.4 400*1.4];
%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECT 1 %%%%%%%%%%%%%%%%%%%%%%
%RADAR
scatter(object1_radar_x,object1_radar_y,"filled","MarkerFaceColor",[0.6350 0.0780 0.1840])
hold on
%CAMERA
scatter(object1_cam_x,object1_y,"filled","MarkerFaceColor",[0.4660 0.6740 0.1880]	)
%BAYES
%scatter(object_fusion_x1,object_fusion_y1+5.5,"filled")

plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',2)

%plot(-1*route_p1(1,:),route_p1(2,:)+5.5,'red','LineWidth',2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECT 2 %%%%%%%%%%%%%%%%%%%%%%
%RADAR
scatter(object2_radar_x,object2_radar_y,"filled","MarkerFaceColor",[0.9290 0.6940 0.1250])
%CAMERA
scatter(object2_cam_x,object2_y,"filled","MarkerFaceColor",[0.3010 0.7450 0.9330])
%BAYES
%scatter(object_fusion_x2,object_fusion_y2+5.5,"filled")

plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',2)

%plot(-1*route_p2(1,:),route_p2(2,:)+5.5,'blue','LineWidth',2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lgd=legend('radar 1','camera 1','Ground Truth 1', ...
          'radar 2','camera 2','Ground Truth 2','Location','Best');
%lgd=legend('radar1','camera1','bayes1','Ground Truth 1','Ground Truth 1', ...
%          'radar2','camera2','bayes2','Ground Truth 2','Ground Truth 2','Location','Best');
title('Scenario 1 Raw Data')
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Fusion %%%%%%%%%%%%%%%%%%%%%%%%%%
f2 = figure(2);
f2.Position = [700 100 550*1.4 400*1.4];
%BAYES
scatter(object_fusion_x1,object_fusion_y1,"filled","MarkerFaceColor",[0.8500 0.3250 0.0980]	)
hold on
plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',2)
%BAYES
scatter(object_fusion_x2,object_fusion_y2,"filled","MarkerFaceColor",[0 0.4470 0.7410]	)
plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',2)
title('Scenario 1 Fusion')
lgd=legend('Object 1 Fusion','Ground Truth 1','Object 2 Fusion','Ground Truth 2','Location','Best');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold off