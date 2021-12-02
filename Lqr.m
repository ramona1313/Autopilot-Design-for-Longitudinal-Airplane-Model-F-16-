%Longitudinal Equations after linearization

A_lon =[-0.0236   -2.3340 -32.0901   -2.2985;-0.0004   -0.4492 0.0023    0.9474;0 0 0 0.9146;0 0.7848 0 -0.6032];
B_lon=[0   -0.0011;0   -0.0009; 0 0;0 -0.0610 ];
C_lon=eye(4);
D_lon=[0 0;0 0;0 0;0 0];
sys_lon=ss(A_lon,B_lon(:,2),C_lon,D_lon(:,2));


%% LQR
Q =diag([0 1/(pi/4)^2 1/(pi/4)^2 1/(pi/4)^2]);
R =0.005;
K=lqr(sys_lon,Q,R);
A=A_lon-B_lon(:,2)*K;
sys1=ss(A,B_lon(:,2),C_lon,D_lon(:,2));


%% Kalman filter
G=[38.3471; 38.3471; 38.3471; 38.3471];
Qn=1
Rn=eye(4)
L =lqe(A_lon,G,C_lon,Qn,Rn)

%Calculating Nbar(Constant gain factor for Reference tracking)
% Use C =[0 0 1 0]; for pitch control
% C=[0 1 0 0]; for angle of attack control
%C= [0 1 0 1]; fro pitch rate control
alpha =inv(-[1 0 0 0]*(inv(A_lon-B_lon(:,2)*K)*B_lon(:,2)));

A_cl =[A_lon -B_lon(:,2)*K; L*C_lon A_lon-L*C_lon-B_lon(:,2)*K]
B_cl =[B_lon(:,2)*alpha; B_lon(:,2)*alpha];
C_cl = [C_lon zeros(4,4); zeros(4,4) C_lon]
D_cl =[ zeros(8,1)];

t=0:0.1:160;
%Setting input reference for different output
u=650*ones(1,length(t));
%initial conditions 
x0 =x0(4);
%initial condition for Estimator
x0=[x0; 0; 0; 0; 0];
%initial condition for airspeed
x0 = [597.7454
   -0.5093
   -0.4062
    0.0001
  597.7454
   -0.5093
   -0.4062
    0.0001];

%Close loop system for state feedback estimator
sys = ss(A_cl,B_cl,C_cl,D_cl);
[y,t]=lsim(sys,u,t,x0)
%[y,t]=step(sys);

%% Plotting 

subplot(221)
plot(t,y(:,1),t,y(:,5),'--')
ylabel('v(ft/sec)')
xlabel('time(sec)')
title('Airspeed')
subplot(222)
plot(t,(180/pi)*y(:,2),t,(180/pi)*y(:,6),'--')
title('Angle of attack')
ylabel('alpha(deg)')
xlabel('time(sec)')
subplot(223)
plot(t,(180/pi)*y(:,3),t,(180/pi)*(y(:,7)),'--')
title('Pitch')
legend('True state','Estimated')
ylabel('theta(deg)')
xlabel('time(sec)')
subplot(224)
plot(t,(180/pi)*y(:,4),t,(180/pi)*y(:,8),'--')
ylabel('q(deg/sec)')
xlabel('time(sec)')
title('Pitch rate')
hold on;
yline(650);
legend('True state','Estimated','Desired')
%step(sys_lon)

% figure(2)
% bode(sys_lon)
