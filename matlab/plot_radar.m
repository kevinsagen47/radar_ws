object1_cam_x=radar_cam_callib(:,1);
object1_radar_x=radar_cam_callib(:,3);
object1_y=radar_cam_callib(:,4);

object2_cam_x=radar_cam_callib(:,5);
object2_radar_x=radar_cam_callib(:,7);
object2_y=radar_cam_callib(:,8);

cam_cov1=cov(object1_cam_x);
radar_cov1=cov(object1_radar_x);
x_bayes1=((object1_radar_x/radar_cov1^2)+(object1_cam_x/cam_cov1^2))/((1/radar_cov1^2)+(1/cam_cov1^2));

cam_cov2=cov(object2_cam_x);
radar_cov2=cov(object2_radar_x);
x_bayes2=((object2_radar_x/radar_cov2^2)+(object2_cam_x/cam_cov2^2))/((1/radar_cov2^2)+(1/cam_cov2^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECT 1 %%%%%%%%%%%%%%%%%%%%%%
%RADAR
scatter(object1_radar_x,object1_y,"filled")
hold on
%CAMERA
scatter(object1_cam_x,object1_y,"filled")
%BAYES
scatter(x_bayes1,object1_y+5.5,"filled")

plot(-1*route_p1(1,:),route_p1(2,:),'red','LineWidth',1)

plot(-1*route_p1(1,:),route_p1(2,:)+5.5,'red','LineWidth',1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECT 2 %%%%%%%%%%%%%%%%%%%%%%
%RADAR
scatter(object2_radar_x,object2_y,"filled")
%CAMERA
scatter(object2_cam_x,object2_y,"filled")
%BAYES
scatter(x_bayes2,object2_y+5.5,"filled")

plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',1)

plot(-1*route_p2(1,:),route_p2(2,:)+5.5,'blue','LineWidth',1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%fontsize(lgd,14,'points')
%hold on
%scatter(radar_kf_x,radar_kf_y,"filled")
%hold on
%scatter(radar_camera_x,radar_camera_y,"filled")

%plot(-1*route_p2(1,:),route_p2(2,:),'blue','LineWidth',1)
lgd=legend('radar1','camera1','bayes1','Ground Truth 1','Ground Truth 1', ...
          'radar2','camera2','bayes2','Ground Truth 2','Ground Truth 2','Location','Best');

hold off