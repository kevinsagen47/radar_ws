%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%
object_fusion_x1=all_scene_1(:,6);
object_fusion_y1=all_scene_1(:,7);
object_fusion_x2=all_scene_1(:,17);
object_fusion_y2=all_scene_1(:,18);

f1 = figure(1);
clf(f1,'reset')
f1.Position = [700 100 550*1.4 400*1.4];
%BAYES
scatter(object_fusion_x1,object_fusion_y1,"filled","MarkerFaceColor",[0.8500 0.3250 0.0980]	)
hold on
plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',2)
%BAYES
scatter(object_fusion_x2,object_fusion_y2,"filled","MarkerFaceColor",[0 0.4470 0.7410]	)
plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',2)
draw_sensor_0_0()
title('Scenario 1 Fusion')
lgd=legend('Object 1 Fusion','Ground Truth 1','Object 2 Fusion','Ground Truth 2','Location','Best');