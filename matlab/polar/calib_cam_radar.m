f2=figure(1);
clf(f2,'reset')
f2.Position  = [100 100 550*1.4 400*1.4];
x = u_vs_theta2(:,1);
y = u_vs_theta2(:,2);
[p,S] = polyfit(x,y,1); 
[y_fit,delta] = polyval(p,x,S);
plot(x,y,'bo')
hold on
plot(x,y_fit,'r-')
%plot(x,y_fit+2*delta,'m--',x,y_fit-2*delta,'m--')
title('Homography result of Camera u to Radar Azimuth Data')
%legend('Camera Homography Result','Ground Truth','95% Prediction Interval')
legend('Homography Result','Ground Truth')
xlabel('Camera u (pixel)')
ylabel('Radar Azimuth (rad)')