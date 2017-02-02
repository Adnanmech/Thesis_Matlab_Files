% A = meshgrid(linspace(0, 2*pi, 40), linspace(0, 2*pi, 40)) ;
% 
% X = .3 .* cos(A);
% Y = .3 .* sin(A);
% 
% Z = meshgrid(linspace(-.3, .3, 40), linspace(-.3, .3, 40))';
% 
% surf(Z, Y, X), axis equal

ax = axes('XLim',[-2 1],'YLim',[-2 1],'ZLim',[-1 1]);
view(3); grid on; axis equal

[x y z] = cylinder([.3 0]);
h(1) = surface(x,y,z,'FaceColor','red');
h(2) = surface(x,y,-z,'FaceColor','green');
h(3) = surface(z,x,y,'FaceColor','blue');
h(4) = surface(-z,x,y,'FaceColor','cyan');
h(5) = surface(y,z,x,'FaceColor','magenta');
h(6) = surface(y,-z,x,'FaceColor','yellow');

t1 = hgtransform('Parent',ax);
t2 = hgtransform('Parent',ax);

set(gcf,'Renderer','opengl')

set(h(1:6),'Parent',t1)
h2 = copyobj(h(1:2),t2);

Txy = makehgtform('translate',[-1.5 -1.5 0]);
set(t2,'Matrix',Txy)
drawnow

% Rotate 10 times (2pi radians = 1 rotation) 
for r = 1:.1:20*pi 
	% Form z-axis rotation matrix 
	Rz = makehgtform('xrotate',r);
    RRz = makehgtform('zrotate',(2*r));
	% Set transforms for both hgtransform objects 
	set(t1,'Matrix',Rz)
	set(t2,'Matrix',Txy*(RRz))
	drawnow
    %pause(.);
end