%plot all 3 scenarios of fused results
light_orange=[0.9290 0.6940 0.1250]	;
blue=[0 0.4470 0.7410];
%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%
object_fusion_x1=all_scene_1(:,6);
object_fusion_y1=all_scene_1(:,7);
object_fusion_x2=all_scene_1(:,17);
object_fusion_y2=all_scene_1(:,18);

object_fusion_x1=scene_1_cam_radar_fusion(:,4);
object_fusion_y1=scene_1_cam_radar_fusion(:,5);
object_fusion_x2=scene_1_cam_radar_fusion(:,9);
object_fusion_y2=scene_1_cam_radar_fusion(:,10);

f1 = figure(1);
clf(f1,'reset')
f1.Position = [700 100 550*1.4 400*1.4];
%BAYES
scatter(object_fusion_x1,object_fusion_y1,"filled","MarkerFaceColor",blue	)
hold on
plot(-1*route3_p1(1,:),route3_p1(2,:),'blue','LineWidth',2)
%BAYES
scatter(object_fusion_x2,object_fusion_y2,"filled","MarkerFaceColor",light_orange	)
plot(-1*route3_p2(1,:),route3_p2(2,:),'red','LineWidth',2)
draw_sensor_0_0()
title('EKF Output in Cartesian Scenario 1 Fusion')
xlabel('x-axis (m)')
ylabel('y-axis (m)')
lgd=legend('Object 1 Fusion','Ground Truth 1','Object 2 Fusion','Ground Truth 2','Location','Best','NumColumns',2);

%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 2 %%%%%%%%%%%%%%%%%%%%%%%%%
object_fusion_x1=all_scene_2(:,6);
object_fusion_y1=all_scene_2(:,7);
object_fusion_x2=all_scene_2(:,17);
object_fusion_y2=all_scene_2(:,18);

object_fusion_x1=scene_2_cam_radar_fusion(:,4);
object_fusion_y1=scene_2_cam_radar_fusion(:,5);
object_fusion_x2=scene_2_cam_radar_fusion(:,9);
object_fusion_y2=scene_2_cam_radar_fusion(:,10);


f1 = figure(2);
clf(f1,'reset')
f1.Position = [700 100 550*1.4 400*1.4];
%BAYES
scatter(object_fusion_x1,object_fusion_y1,"filled","MarkerFaceColor",blue	)
hold on
plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',2)
%BAYES
scatter(object_fusion_x2,object_fusion_y2,"filled","MarkerFaceColor",light_orange	)
plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',2)
draw_sensor_0_0()
title('EKF Output in Cartesian Scenario 2')
xlabel('x-axis (m)')
ylabel('y-axis (m)')
lgd=legend('Object 1 Fusion','Ground Truth 1','Object 2 Fusion','Ground Truth 2','Location','Best','NumColumns',2);


%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 3 %%%%%%%%%%%%%%%%%%%%%%%%%
object_fusion_x1=scene_3_cam_radar_fusion(:,4);
object_fusion_y1=scene_3_cam_radar_fusion(:,5);
object_fusion_x2=scene_3_cam_radar_fusion(:,9);
object_fusion_y2=scene_3_cam_radar_fusion(:,10);

f1 = figure(3);
clf(f1,'reset')
f1.Position = [700 100 550*1.4 400*1.4];
%BAYES
scatter(object_fusion_x1,object_fusion_y1,"filled","MarkerFaceColor",blue	)
hold on
plot(-1*route1_p1(1,:),route1_p1(2,:),'blue','LineWidth',2)
%BAYES
scatter(object_fusion_x2,object_fusion_y2,"filled","MarkerFaceColor",light_orange	)
plot(-1*route1_p2(1,:),route1_p2(2,:),'red','LineWidth',2)
draw_sensor_0_0()
title('EKF Output in Cartesian Scenario 3')
xlabel('x-axis (m)')
ylabel('y-axis (m)')
lgd=legend('Object 1 Fusion','Ground Truth 1','Object 2 Fusion','Ground Truth 2','Location','Best','NumColumns',2);
