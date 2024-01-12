%test platform for bounded line

x = double(theta_x_axis);%linspace(0, 2*pi, 50);
y1 = double(cam_theta1);%sin(x);
%y2 = cos(x);
%b = [ones(size(y1))*0.2; rand(size(y1))*.5+.5]';
%e1 = double(cam_min_theta1)'; double(cam_max_theta1)']';
e1 = (double(cam_max_theta1)-double(cam_min_theta1))/2';
%e1=[ones(size(y1))*0.2;ones(size(y1))*0.2];
%e1 = rand(size(y1))*.5+.5;
%e1=b;
e2 = [.25 .5];

%ax(1) = subplot(2,2,1);
figure
%[l,p] = boundedline(x, y1, e1, '-b*', x, y2, e2, '--ro');
%[l,p] = boundedline(x, y1, e1, 'o', x, y2, e2, 'r');
%hnew(il) = line(xy{1}, xy{2}, 'parent', ax,'linestyle','none','marker','o', 'color', col,'MarkerFaceColor',col);
[l,p] = boundedline(x, y1, e1(:,1), 'bo');
line1 = outlinebounds(l,p);
set([l,p],'linestyle','none');
title('Opaque bounds, with outline');
axis tight;
%{
ax(2) = subplot(2,2,2);
boundedline(x, [y1;y2], rand(length(y1),2,2)*.5+.5, 'alpha');
title('Transparent bounds');
axis tight;

ax(3) = subplot(2,2,3);
boundedline([y1;y2], x, e1(1), 'orientation', 'horiz')
title('Horizontal bounds');
axis tight;

ax(4) = subplot(2,2,4);
boundedline(x, repmat(y1, 4,1), permute(0.5:-0.1:0.2, [3 1 2]), ...
'cmap', cool(4), ...
'transparency', 0.5);
title('Multiple bounds using colormap');

set(ax([1 2 4]), 'xlim', [0 2*pi]);
set(ax(3), 'ylim', [0 2*pi]);
axis tight;
%}