[1,64,89,97],-1*theta_gt_p1_s1
coefficients = polyfit([theta2_gt_p1_s1(1,1), theta2_gt_p1_s1(1,2)], [theta2_gt_p1_s1(2,1), theta2_gt_p1_s1(2,2)], 1);
a = coefficients (1);
b = coefficients (2);

y_1=@(x)a*x+b;
x_1 = 1:1:70;
f=figure(1)
clf(f,"reset")
plot(x_1,y_1(x_1),'r','LineWidth',3)
hold on;
plot(theta2_gt_p1_s1(1,1:2),theta2_gt_p1_s1(2,1:2),'b')

figure(2)
plot(theta2_gt_p1_s1(1,1:2),theta2_gt_p1_s1(2,1:2),'b')