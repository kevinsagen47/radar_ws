function draw_sensor_0_0()
    %plot([0 0],[0.5 0],'k','LineWidth',2)
    %plot([0 0.25],[0 0],'k','LineWidth',2)
    %text(pi,0,'\leftarrow sin(\pi)')
    %f2=figure(2);
    %clf(f2,'reset')
    %%{
    %drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 )    
    drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );
    x1 = [0 0];
    y1 = [0 0.5];
    %up
    drawArrow(x1,y1,'linewidth',3,'color','k'); hold on
    scatter(0,0,9,'k',"filled")
    %%}
    x2 = [0 0.25];
    y2 = [0 0];
    
    %drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );
    %left
    drawArrow(x2,y2,'linewidth',3,'color','k')
    text(-0.02,0.7,'y')
    text(0.27,0.02,'x')
    text(0.1,0.3,'sensor')
    %ylim([-0.5 6])
    %xlim([-2.5 1.5])
end