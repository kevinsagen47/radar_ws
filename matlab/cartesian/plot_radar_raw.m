%plot raw data of radar
%%{
%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure(1);
clf(f1,'reset')
f1.Position = [100 100 550*1.4 400*1.4];
scatter(raw_radar_scene1(:,1),raw_radar_scene1(:,2),"filled")
hold on
plot(-1*route3_p1(1,:),route3_p1(2,:),'blue','LineWidth',2)
scatter(raw_radar_scene1(:,3),raw_radar_scene1(:,4),"filled")
plot(-1*route3_p2(1,:),route3_p2(2,:),'red','LineWidth',2)
draw_sensor_0_0()
lgd=legend('Radar Object 1',"Ground Truth Object 1",'Radar Object 2',"Ground Truth Object 2","","","",'Location','Best','NumColumns',2);


hold off
xlabel('x(m)')
ylabel('y(m)')
ylim([-0.5 6])

title('Object Tracking Only Radar Scenario 1')
%%}


%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 2 %%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure(2);
clf(f1,'reset')
f1.Position = [100 100 550*1.4 400*1.4];
scatter(raw_radar_scene2(:,1),raw_radar_scene2(:,2),"filled")
hold on
plot(-1*route2_p2(1,:),route2_p2(2,:),'blue','LineWidth',2)
scatter(raw_radar_scene2(:,5),raw_radar_scene2(:,6),"filled")
plot(-1*route2_p1(1,:),route2_p1(2,:),'red','LineWidth',2)
draw_sensor_0_0()
hold off
lgd=legend('Radar Object 1',"Ground Truth Object 1",'Radar Object 2',"Ground Truth Object 2","","","",'Location','Best','NumColumns',2);
xlabel('x(m)')
ylabel('y(m)')
title('Object Tracking Only Radar Scenario 2')

%%%%%%%%%%%%%%%%%%%%%%%%%   SCENARIO 3 %%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure(3);
clf(f1,'reset')
f1.Position = [100 100 550*1.4 400*1.4];
scatter(raw_radar_scene3(:,1),raw_radar_scene3(:,2),"filled")
hold on
plot(-1*route1_p1(1,:),route1_p1(2,:),'blue','LineWidth',2)
scatter(raw_radar_scene3(:,5),raw_radar_scene3(:,6),"filled")
plot(-1*route1_p2(1,:),route1_p2(2,:),'red','LineWidth',2)
draw_sensor_0_0()
hold off
lgd=legend('Radar Object 1',"Ground Truth Object 1",'Radar Object 2',"Ground Truth Object 2","","","",'Location','Best','NumColumns',2);
xlabel('x(m)')
ylabel('y(m)')
title('Object Tracking Only Radar Scenario 3')


