go = [1;2;3];  %starting point
st = [2;3;4];  %ending points
nPoints = 10; 
x = linspace(0,1,nPoints);
A2 = go + x.*(st - go);

route3_p1 = [-2 0 -2 -2; 
              9 7 5 9]*0.6;
route3_p2 = [3 1 3 3; 
             9 7 5 9]*0.6;

theta_gt_p1_s1=atan2(route3_p1(1,:),route3_p1(2,:));
theta_gt_p2_s1=atan2(route3_p2(1,:),route3_p2(2,:));

theta_gt_p1_s2=atan2(route2_p1(1,:),route2_p1(2,:));
theta_gt_p2_s2=atan2(route2_p2(1,:),route2_p2(2,:));

theta_gt_p1_s3=atan2(route1_p1(1,:),route1_p1(2,:));
theta_gt_p2_s3=atan2(route1_p2(1,:),route1_p2(2,:));
%clf

figure(2)
hold on
plot(1:length(theta_gt_p1_s2),theta_gt_p1_s2,'red')
plot(1:length(theta_gt_p1_s2),theta_gt_p2_s2,'blue')
hold off