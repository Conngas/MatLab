%% 读取数据与处理
data = importdata('kalman_vel_pos.txt');

t        = data.data(:, 1);
accx_dji = data.data(:, 2);
accy_dji = data.data(:, 3);
gpsx_dji = data.data(:, 5);
gpsy_dji = data.data(:, 6);
velx_dji = data.data(:, 7);
vely_dji = data.data(:, 8);
posx_dji = data.data(:, 9);
posy_dji = data.data(:, 10);

% 加速度积分计算速度、位移
dt = [0; t(2:end)-t(1:end-1)];
velx_int = velx_dji(1) + cumsum(accx_dji .* dt);   % 积分计算速度
vely_int = vely_dji(1) + cumsum(accy_dji .* dt);   % 积分计算速度
posx_int = posx_dji(1) + cumsum(velx_int .* dt);   % 积分计算位移
posy_int = posy_dji(1) + cumsum(vely_int .* dt);   % 积分计算位移

% GPS直接计算位移
gpsx0 = gpsx_dji(1);
gpsy0 = gpsy_dji(1);

earth_radius = 6378137.0;
posx_gps = (gpsx_dji - gpsx0) * pi / 180 * earth_radius .* cos(gpsy_dji*pi/180);        % 直接只用GPS估算的位移
posy_gps = (gpsy_dji - gpsy0) * pi / 180 * earth_radius;                                % 直接使用GPS估算的位移

posx_gps = posx_gps + posx_dji(1);      % 加上初始位移
posy_gps = posy_gps + posy_dji(1);      % 加上初始位移

% 卡尔曼滤波计算速度、位移
[velx_km, posx_km] = kalman(t, accx_dji, posx_gps);
[vely_km, posy_km] = kalman(t, accy_dji, posy_gps);

%% 结果分析
%（位置）
figure(1);
plot(posx_int, posy_int);hold on
plot(posx_gps, posy_gps, 'b.');
plot(posx_dji, posy_dji,'g');
plot(posx_km, posy_km,'r'); hold off
legend('加速度积分', 'GPS位移', 'DJI结果','Kalman结果');
title('卡尔曼滤波估计位移');xlabel('x方向位移 (m)');ylabel('y方向位移 (m)');
text(posx_dji(1), posy_dji(1), '起始点');
text(posx_dji(end),posy_dji(end), '终止点');

%（速度）
figure(2);subplot(211);
plot(t-t(1), velx_int);hold on
plot(t-t(1), velx_dji);
plot(t-t(1), velx_km);hold off
legend('加速度积分','DJI结果','Kalman结果');
title('卡尔曼滤波估计速度');ylabel('x轴速度 (m/s)');

subplot(212);
plot(t-t(1), vely_int);hold on
plot(t-t(1), vely_dji);
plot(t-t(1), vely_km);hold off
legend('加速度积分','DJI结果','Kalman结果');
xlabel('时间t (s)');ylabel('y轴速度 (m/s)');

%% 卡尔曼滤波器
% 输入: t, 时间戳(s)
%       acc, 无人机所受合力在地球坐标系下产生的加速度(m/s^2)
%       gps, 地球坐标系下使用GPS计算的位移(m)
% 输出: vel, 地球坐标系下的速度(m/s)
%       pos, 地球坐标系下的位移(m)

function [vel, pos] = kalman(t, acc, gps)

H = [1 0];                              % 转换矩阵
Q = [1e-4 0; 0 5e-4];                   % 过程噪声协方差，估计一个
R = 10;
P = eye(2);                             % 初始值为 1（可为非零任意数） 
N = length(acc);
x = zeros(2, N);                        % 存储滤波后的数据，分别为位移、速度信息

for k=2:N
    dt = t(k) - t(k-1);
    A = [1 dt; 0 1];                            % 状态转移矩阵
    B = [1/2*dt^2; dt];                         % 输入控制矩阵
    x(:,k) = A * x(:,k-1) + B*acc(k,1);         % 卡尔曼公式1
    P = A * P * A' + Q;                         % 卡尔曼公式2
    K = P*H' * inv(H*P*H' + R);                 % 卡尔曼公式3                                       
    x(:,k) = x(:,k) + K * (gps(k)-H*x(:,k));    % 卡尔曼公式4
    P = (eye(2)-K*H) * P;                       % 卡尔曼公式5
end

pos = x(1,:)';
vel = x(2,:)';

end

%%
