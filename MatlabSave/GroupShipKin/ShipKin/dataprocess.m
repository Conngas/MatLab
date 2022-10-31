%从header文件中提取数据
clear
close all
filename = './txt-format/2021-08-12 12_27_07-iip-header.txt';
data = load(filename);


f = 5;               %Hz
h = 1/f;       %time step (s)
t = h*data(:,6);       %time serial
[row,~] = size(data);%total number of data from senor
long = data(:,31);   %longtitude
lat = data(:,32);    %latitude
BasePoint = [long(1) lat(1)];%BasePoint
psi = data(:,34);    %head angle
r = data(:,35);      %Yaw angle velocity
theta = data(:,37);  %pitch
q = data(:,38);      %pitch angle velocity
phi = data(:,40);    %roll angle
p = data(:,41);      %roll angle velocity
U = data(:,43);      %ship real velocity
U_angle = data(:,44);%angle between U and X axis in fixed oridinate

velocity_angle = U_angle - psi; %angle between U direction and ship head direction
u = U.*cos(velocity_angle*pi/180); % velocity of x axis in ship coordinate 
v = U.*sin(velocity_angle*pi/180); % velocity of y axis in ship coordinate 

u_r = u.*cos(psi*pi/180) - v.*sin(psi*pi/180);%Transform coordinate 
v_r = u.*sin(psi*pi/180) + v.*cos(psi*pi/180);%Transform coordinate 

% u1 = u_r.*cos(psi*pi/180) + v_r.*sin(psi*pi/180);%Transform coordinate 
% v1 = -u_r.*sin(psi*pi/180) + v_r.*cos(psi*pi/180);%Transform coordinate 

delta_l = data(:,162);%left rudder angle 
delta_r = data(:,161);%right rudder angle 
thrust_l = data(:,157);%left thrust 
thrust_r = data(:,158);%right thrust 

%-----经纬度转换--------%
X = zeros(row,1);
Y = zeros(row,1);
X_r(:,1) = [0 ; 0]; % [x_r ;y_r] distance in reference coordinate
for i= 1:row
    [ang,dist]=ComputeAngDist(BasePoint(1),BasePoint(2),long(i), lat(i));
%     ang = 90.0 - ang;
    X(i)=dist*cos(ang*pi/180.0);
    Y(i)=dist*sin(ang*pi/180.0);
    
    X_r(:,i+1) = euler2([u_r(i); v_r(i)], X_r(:,i), h);
end

%-------绘图--------%


figure(1)
plot(0,0,'b*',X,Y,'k-',X_r(1,:),X_r(2,:),'r.');
legend('starPoint','{\itGPS}','{\itins}')
hold on
hold off
grid on;
xlabel('{\itx/m---East}');
ylabel('{\ity/m---North}');
title('{\itwrt航迹显示}');
pause(0.1);

figure(2)
plot(t,psi,'r.');
grid on;
legend('{\itpsi}')
xlabel('{\itt (s)}'),ylabel('{\itpsi (°)}')
title('艏向角')
pause(0.1);

figure(3)
subplot(1,2,1)
plot(t,u,'k-',t,v,'r-');
grid on;
legend('{\itu}','{\itv}');
title('{\it船体坐标系速度}');
xlabel('{\itt（s）}');
ylabel('{\itvelocity (m/s)}');
subplot(1,2,2)
plot(t,r,'k-');
grid on;
legend('{\itr}');
title('{\it艏向角速度}');
xlabel('{\itt（s）}');
ylabel('{\itr (°/s)}');
pause(0.1);

figure(4)
plot(t,u_r,'b-',t,v_r,'g-');
grid on;
legend('u_r','v_r');
xlabel('{\itt(s)}'),ylabel('{\itvelocity (m/s)}')
title('{\it参考坐标系下无人艇速度}')
pause(0.1);

figure(5)
subplot(1,2,1)
plot(t,theta,'k-');
grid on;
legend('theta');
xlabel('{\itt(s)}'),ylabel('{\ittheta(°)}');
title('{\it俯仰角}');
subplot(1,2,2)
plot(t,q,'r-');
grid on;
legend('q');
xlabel('{\itt(s)}'),ylabel('{\itq(°/s)}');
title('{\it俯仰角速度}');
pause(0.1);

figure(6)
subplot(1,2,1)
plot(t,phi,'k-',t,delta_l,'r-');
grid on
legend('phi','delta');
xlabel('t(s)'),ylabel('{\itphi(°) \itdelta(°)}')
title('{\it横滚角、舵角}')
subplot(1,2,2)
plot(t,thrust_l,'k.',t,thrust_r,'r-');
grid on
legend('thrust_l/°','thrust_r/°')
xlabel('t(s)'),ylabel('thrust(°)')
title('{\it左右推力}')

%---------svae as csv file--------%
% All = [t,X,Y,psi,r,theta,q,phi,p,U,U_angle,delta_l,delta_r,thrust_l,thrust_r];
% csvwrite('test17_turningTest_5.csv',All);

% All = [t,X,Y,psi,r,theta,q,phi,p,u,v,delta_l,delta_r,thrust_l,thrust_r];
% csvwrite('test17_turningTest_5.csv',All);
