% 20191105 bulid by Riondan
if exist('s') 
   fclose(s);
    delete(s);
    clear s 
end
clc;
clear;
close all;
%% Auxiliary variables
global dt;dt = 1/20;%0.1s
% quaternion of sensor frame relative to auxiliary frame
global q0;q0 = 1.0;
global q1;q1 = 0.0;
global q2;q2 = 0.0;
global q3;q3 = 0.0;
%quaternion of sensor frame relative to auxiliary frame 
global dq0;dq0 = 0.0;
global dq1;dq1 = 0.0;
global dq2;dq2 = 0.0;
global dq3;dq3 = 0.0;
global gyro_bias;gyro_bias = [0.0, 0.0, 0.0];
global bFilterInit;bFilterInit = 0; 
global raw_data_count;raw_data_count = 0;   %us 
global acc;acc = [];
global gyro;gyro = [];
global mag;mag = [];

global last_roll;last_roll=0;
global last_pitch;last_pitch=0;
global last_yaw;last_yaw=0;
global acc_offset;acc_offset = [0 0 0];
global gyro_offset;gyro_offset = [0 0 0];
global mag_offset;mag_offset = [0 0 0];

global acc_scal;acc_scal = [1 1 1];
global gyro_scal;gyro_scal = [1 1 1];
global mag_scal;mag_scal = [1 1 1];
%% Calibration parameters
global last_angel;last_angel = 0;
global X;X = [];
global Y;Y = [];
global Z;Z = [];

global X_offset;X_offset = 0;
global Y_offset;Y_offset = 0;
global Z_offset;Z_offset = 0;

global Xmin;Xmin = 0;
global Xmax;Xmax = 0;
global Ymin;Ymin = 0;
global Ymax;Ymax = 0;
global Zmin;Zmin = 0;
global Zmax;Zmax = 0;

global Xscal;Xscal = 1;
global Yscal;Yscal = 1;
global Zscal;Zscal = 1;
%% Three-dimensional map
figure(1);
p = scatter3(X,Y,Z,'EraseMode','background');
% axis([-1 1 -1 1 -1 1]);
axis([-1000 1000 -1000 1000 -1000 1000]);
%daspect([1 1 10]);
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
%% Two-dimensional projection
figure(2);
hold on;
subplot(221);
xy_point = scatter(X,Y);
%axis([-1000 1000 -1000 1000]);
xlabel('x-y');
axis equal;
subplot(222);
xz_point = scatter(X,Z);
%axis([-1000 1000 -1000 1000]);
xlabel('x-z');
axis equal;
subplot(223);
yz_point = scatter(Y,Z);
%axis([-1000 1000 -1000 1000]);
xlabel('y-z');
axis equal;
%% 3D Model loading
[x,y] = meshgrid(-5:0.1:5-0.1,-5:0.1:5-0.1);
img_plane = imread('plane.jpg');
img_plane = im2bw(img_plane,0.5);
img_plane = img_plane * (-1) +1;
img_plane = img_plane * 2;
img_plane = imresize(img_plane,[100 100]);
z = img_plane;
figure(3);
model=mesh(x,y,z);
axis([-10  10 -10 10 -10 10]);
xlabel('X');
ylabel('Y');
zlabel('Z');
%% Error tracking
global yaw_error;
global pitch_error;
global roll_error;
global data_error;
global data_error_t;
global data_error_counter;
data_error_t = 0;
data_error = [ 0  0  0];
data_error_counter = 0;
figure(4);
subplot(311);
hold on;
roll_error = plot(data_error_t,data_error(1));
ylabel('roll/бу');
% axis equal;
grid on;
subplot(312);
hold on;
pitch_error = plot(data_error_t,data_error(2));
ylabel('pitch/бу');
% axis equal;
grid on;
subplot(313);
hold on;
yaw_error = plot(data_error_t,data_error(3));
ylabel('yaw/бу');
% axis equal;
grid on;
%% open series service
devices=IdentifySerialComs();
try
    s=serial(['com',num2str(devices{2}(1))]);
catch
    error('cant serial');
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
s.BytesAvailableFcnMode = 'terminator';
s.BytesAvailableFcn = {@callback,p,model,xy_point,xz_point,yz_point};
global last_num;last_num = 0;
global num;num = 0;
fopen(s);
Model_Selection = 1;
if Model_Selection == 1
    %% Software simulation part (fixed input)
    for i = 1:1000/dt
        send_data = ['0', ' ', '0' ,' ' ,'0' ,' ', '0' ,' ' ,'0', ' ', '0', ' ',  '-182', ' ', '-207' ,' ',  '-104'];%512   20200108   acc gyro mag
        fprintf(s,send_data);
        if num - last_num > 0
           last_num = last_num + 1;
            attitude_plane(p,model,xy_point,xz_point,yz_point); 
        end
        pause(dt);
    end
elseif Model_Selection == 2
    %% Software simulation part (file input)
    data=load('data_acc.mat');
    data_acc=data.acc;
    data=load('data_gyro.mat');
    data_gyro = data.gyro;
    data=load('data_mag.mat');
    data_mag = data.mag;
    for i = 1:length(data_acc(:,1))
       send_data = [num2str(data_acc(i,1)) ' ' num2str(data_acc(i,2))  ' ' num2str(data_acc(i,3)) ' ' num2str(data_gyro(i,1)) ' ' num2str(data_gyro(i,2))  ' ' num2str(data_gyro(i,3))  ' ' num2str(data_mag(i,1)) ' ' num2str(data_mag(i,2))  ' ' num2str(data_mag(i,3))];
       fprintf(s,send_data);
       pause(dt);
    end
else
    %% Hardware real-time processing part
    while(1)
        if num - last_num > 0
           last_num = last_num + 1;
           attitude_plane(p,model,xy_point,xz_point,yz_point); 
        end
    %     pause(1/10000);
    end
end
%% close series service
pause;
fclose(s);
delete(s);
clear s