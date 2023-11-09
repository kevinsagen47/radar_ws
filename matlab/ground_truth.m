%%bound
bound = [[-4 0]
         [-4 11]
         [4 11]
         [4 0]];

bound_x = [-4 -4 4 4 -4]*0.6;
bound_y = [0 11 11 0 0]*0.6;

hold on
plot(bound_x,bound_y)
hold off

%%route1
route1_p1 = [-2 0 -2 0 -2; 
              9 8 6 4 4]     *0.6;
route1_p2 = [2 0 2 0 2; 
             9 7 6 6 4]*0.6;


%%route2
route2_p1 = [2 -2 -2 2; 4 4 6 6]*0.6;
route2_p2 = [-2 2 2 -2; 9 9 7 7]*0.6;

%%route3
route3_p1 = [-2 0 -2 -2; 
              9 7 5 9]*0.6;
route3_p2 = [3 1 3 3; 
             9 7 5 9]*0.6;

%%plot  route 1/2/3
route_p1 = route1_p1;
route_p2 = route1_p2;
    
hold on
scatter(route_p1(1,:),route_p1(2,:),'red')
plot(route_p1(1,:),route_p1(2,:),'red')
scatter(route_p2(1,:),route_p2(2,:),'blue')
plot(route_p2(1,:),route_p2(2,:),'blue')
hold off