%%Testing

%VMC(1:7,1)

myaxes = axes('xlim',[-10 10],'ylim',[-10 40], 'zlim',[0 2]);
view(-14,14);        % VIEW(3) sets the default 3-D view, AZ = -37.5, EL = 30
grid on;
axis equal;
%hold on
xlabel('x')
ylabel('y')
zlabel('z')

%% 1.1 Create Road lines for reference
p=0;
for s = 0:100
    p = s*3;
    line([-5 5],[p p],'color',[0 0 1]);
end

cameraposition = campos
cameraposition_x = cameraposition(1)
cameraposition_y = cameraposition(2)
cameraposition_z = cameraposition(3)

cameratarget = camtarget
cameratarget_x = cameratarget(1)
cameratarget_y = cameratarget(2)
cameratarget_z = cameratarget(3)


for f = 0:30
    %q = f*5;
    cameratarget_y = cameratarget_y+1;
    cameraposition_y = cameraposition_y+1;
    
    camtarget([cameratarget_x,cameratarget_y,cameratarget_z])
    campos([cameraposition_x,cameraposition_y,cameraposition_z])
    
    pause(0.1)
end

for f = 0:30
    %q = f*5;
    %camtarget([camera_target])
    %camdolly(.1,.1,0,'movetarget')
    
    %myaxes = axes('xlim',[-10 10],'ylim',[-10+f 40+f], 'zlim',[0 2]);
    pause(0.1)
end

%camdolly(dx,dy,dz,'fixtarget')
    