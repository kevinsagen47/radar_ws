%plot RMSE vs frame
f=figure(1)
clf(f,'reset')
f.Position = [700 100 550*1.4 400*1.4];
m_red=[0.8500 0.3250 0.0980];
m_blue=[0 0.4470 0.7410];
plot(RMSE_person1_scene1,'LineWidth',2,'color',m_blue);
hold on
plot(RMSE_person2_scene1,'LineWidth',2,'color',m_red);
ylim([0,0.8])
lgd=legend('Object 1','Object 2','Location','Best');
xlabel('Frame (#)')
ylabel('Error (m)')
title('Fused Scenario 1 Cartesian Error')
hold off