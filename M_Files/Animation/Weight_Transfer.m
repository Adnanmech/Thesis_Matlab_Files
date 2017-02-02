%clear variables
clear all


%weight Transfer
%tire locs symbols
a = sym('a');
b = sym('b');
h = sym('h');
tf = sym('tf');
tr = sym('tr');
g  = sym('g');
m  = sym('m');

%forces symbols
fx  = sym('fx');
fy  = sym('fy');
fz  = sym('fz');

f1x = sym('f1x');
f1y = sym('f1y');
f1z = sym('f1z');

f2x = sym('f2x');
f2y = sym('f2y');
f2z = sym('f2z');

f3x = sym('f3x');
f3y = sym('f3y');
f3z = sym('f3z');

f4x = sym('f4x');
f4y = sym('f4y');
f4z = sym('f4z');

%setting coordinates of tire loactions
tirelocs = [[a, tf, h]; [-b, tf, h];[-b, -tf, h]; [a, -tf, h]];

%Forces acting on each tire.    f1 = FR,
%                               f2 = RR, 
%                               f3 = FL, 
%                               f4 = RL. 
forces = [[f1x, f1y, -f1z]; [f2x, f2y, -f2z]; [f3x, f3y, -f3z]; [f4x, f4y, -f4z]];

%Cross Product of torques
torques = cross(tirelocs, forces)

%add X,Y,Z components together for combined forces
sum_of_torques = [sum(torques(:,1)) sum(torques(:,2)) sum(torques(:,3))]

%display x,y forces
sum_of_torques(1)
sum_of_torques(2)

%Let sum_of_torques(1)=0 and sum_of_torques(2)=0. We assume X and Y torques are in equilibrium since car is not flipping. 
torque_equations = [[sum_of_torques(1),sum_of_torques(2)]]

%%%%%%%%%%%% DIDNT WORK %%%%%%%%%%%%%
%%%%syms a b h tf tr f1x f1y f1z f2x f2y f2z f3x f3y f3z f4x f4y f4z
%%%%solvedF1x = solve('forques1(1)=0', 'f1z')
%%%%solvedF1x = solve('f4z*tf - f2y*h - f3y*h - f4y*h - f1z*tf - f1y*h - f2z*tr + f3z*tr=0', 'f1z')

%%%%solvedF1x = solve('forques1(1)=0', 'f1z')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Variable substitution 
%fx = f1x + f2x + f3x + f4x;
%fy = f1y + f2y + f3y + f4y;
%fz = f1z + f2z + f3z + f4z;

%
solved_torque_equations12 = solve(torque_equations, f1z, f2z)
solved_torque_equations34 = solve(torque_equations, f3z, f4z)


%display equations formed
solved_torque_equations12.f1z
solved_torque_equations12.f2z
solved_torque_equations34.f3z
solved_torque_equations34.f4z

%eq1
%factored = -(a*f4z*tr + factor(b*f1y*h + b*f2y*h + b*f3y*h + b*f4y*h) - factor(b*f4z*tf - 2*b*f3z*tr) + factor(f1x*h*tr + f2x*h*tr + f3x*h*tr + f4x*h*tr))/(a*tr + b*tf)

%eq2
%(2*a*f4z*tf + factor( - a*f1y*h - a*f2y*h - a*f3y*h - a*f4y*h) + a*f3z*tr - b*f3z*tf + factor(f1x*h*tf + f2x*h*tf + f3x*h*tf + f4x*h*tf))/(a*tr + b*tf)
%(h*tf*(fx) - a*h*(fy) + 2*a*f4z*tf + a*f3z*tr - b*f3z*tf)/(a*tr + b*tf)
%((- b*f3z + h*fx)*tf - a(h*fy - 2*f4z*tf - f3z*tr))/(a*tr + b*tf)

%eq3
%(a*f1y*h + a*f2y*h + a*f3y*h + a*f4y*h + 2*a*f1z*tf + a*f2z*tr - b*f2z*tf + f1x*h*tf + f2x*h*tf + f3x*h*tf + f4x*h*tf)/(a*tr + b*tf)
%(- b*f2z*tf + h*tf*(fx) + a*h*(fy) + 2*a*f1z*tf + a*f2z*tr)/(a*tr + b*tf)
%((- b*f2z + h*fx)*tf + a*(fy*h + 2*f1z*tf + f2z*tr))/(b*tf + a*tr)

%eq4
%(b*f1y*h + b*f2y*h + b*f3y*h + b*f4y*h - a*f1z*tr + b*f1z*tf + 2*b*f2z*tr - f1x*h*tr - f2x*h*tr - f3x*h*tr - f4x*h*tr)/(a*tr + b*tf)
%(-( a*f1z + fx*h)*tr + b*(fy*h + f1z*tf + 2*f2z*tr))/(b*tf + a*tr)

%final forms of equations
%f1z = -((a*f4z + fx*h)*tr + b*(fy*h - f4z*tf - 2*f3z*tr))/(b*tf + a*tr);
%f2z = ((- b*f3z + fx*h)*tf - a*(fy*h - 2*f4z*tf - f3z*tr))/(b*tf + a*tr);
%f3z = ((- b*f2z + fx*h)*tf + a*(fy*h + 2*f1z*tf + f2z*tr))/(b*tf + a*tr);
%f4z = (-( a*f1z + fx*h)*tr + b*(fy*h + f1z*tf + 2*f2z*tr))/(b*tf + a*tr);

%Cross Rations of weights are equal! Weight jacking of car is symmetric! 
%For instance, the ratio of left to right is the same at front as at rear, or, equivalently, 
%that the ratio of front to rear is the same on left as at right. These conditions yield another equation.

%eq1 = sym('eq1');
%eq2 = sym('eq2');

eq1 = f1z*f3z == f2z*f4z

%One more equation from force equilibrium: the sum of all vertical loads equals the weight of the car.
eq2 = m*g == (f1z + f2z + f3z + f4z)


solved34 = solve(eq1, eq2, f3z, f4z)
%solve(f1z*f3z == f2z*f4z,m*g == (f1z + f2z + f3z + f4z),f3z,f4z)

%f1z = -((a*f4z + fx*h)*tr + b*(fy*h - f4z*tf - 2*f3z*tr))/(b*tf + a*tr);
%f2z = ((- b*f3z + fx*h)*tf - a*(fy*h - 2*f4z*tf - f3z*tr))/(b*tf + a*tr);
%f3z = ((- b*f2z + fx*h)*tf + a*(fy*h + 2*f1z*tf + f2z*tr))/(b*tf + a*tr);
%f4z = (-( a*f1z + fx*h)*tr + b*(fy*h + f1z*tf + 2*f2z*tr))/(b*tf + a*tr);

solved34.f3z
solved34.f4z

FF = (-(solved_torque_equations12.f1z*(solved_torque_equations12.f1z + solved_torque_equations12.f2z - g*m))/(solved_torque_equations12.f1z + solved_torque_equations12.f2z))

simple(FF)
%FF_funct = matlabfunction(FF(a, b, h, tf, tr, f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z, f4x, f4y, f4z))

%normal(FF_funct)








