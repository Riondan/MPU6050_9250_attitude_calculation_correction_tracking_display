function attitude_plane(p,model,xy_point,xz_point,yz_point)  
    global num;
    global last_num;
    global acc_offset;
    global gyro_offset;
    global mag_offset;
    global acc_scal;
    global gyro_scal;
    global mag_scal;
    global q0q0;
    global q0q1;
    global q0q2;
    global q0q3;
    global q1q1;
    global q1q2;
    global q1q3;
    global q2q2;
    global q2q3;
    global q3q3;
	global dt;
    global acc;
    global gyro;
    global mag;
    global last_roll;
    global last_pitch;
    global last_yaw;
    global yaw_error;
    global pitch_error;
    global roll_error;
    global data_error;
    global data_error_t;
    global data_error_counter;
    %% Calculate gyroscope data offset
    if last_num == 10
        gyro_offset(1) = mean(gyro(1:last_num,1));
        gyro_offset(2) = mean(gyro(1:last_num,2));
        gyro_offset(3) = mean(gyro(1:last_num,3));
        gyro_scal(1) = 1;
        gyro_scal(2) = 1;
        gyro_scal(3) = 1;
    end
    %% Accelerometer data correction
    if last_num == 10
        %% Ellipsoid fitting
%         [Center,Scale_axis] = fit_elliposoid9(acc);
%         acc_offset(1) = Center(1);
%         acc_offset(2) = Center(2);
%         acc_offset(3) = Center(3);
%         %recipNorm = invSqrt(Scale_axis(1) * Scale_axis(1) + Scale_axis(2) * Scale_axis(2) + Scale_axis(3) * Scale_axis(3));
%         acc_scal(1) = 1;
%         acc_scal(2) = Scale_axis(1)/Scale_axis(2);
%         acc_scal(3) = Scale_axis(1)/Scale_axis(3);         
        %% Maximum/Minimum Fitting
%         [Xmax,~]=max(acc(:,1));
%         [Xmin,~]=min(acc(:,1));
%         [Ymax,~]=max(acc(:,2));
%         [Ymin,~]=min(acc(:,2));
%         [Zmax,~]=max(acc(:,3));
%         [Zmin,~]=min(acc(:,3));
%         acc_offset(1) = (Xmax+Xmin)/2;
%         acc_offset(2) = (Ymax+Ymin)/2;
%         acc_offset(3) = (Zmax+Zmin)/2;
%         acc_scal(1) = 1;
%         acc_scal(2) = ((Xmax-Xmin)/(Ymax-Ymin));
%         acc_scal(3) = ((Xmax-Xmin)/(Zmax-Zmin)); 
        %%  function calculated Fitting
        % 16g
%         acc_offset = [-1.510891199238572,-0.739295154164768,-3.683959543156365];
%         acc_scal = [1,1.018147965088875,1.003624584550016];
        %8g
%         acc_offset = [-0.404055071516353,-1.020814764446060,0.023521012317164];
%         acc_scal = [1,1.137499297809236,1.092668010644310];
        %4g
        acc_offset = [-0.592469334455157,-0.283369702801081,0.065344716203899];
        acc_scal = [1,1,1.074622952984272];   
        %% Accelerometer data fitting
        acc(1:last_num,1) = (acc(1:last_num,1)-acc_offset(1))*acc_scal(1);
        acc(1:last_num,2) = (acc(1:last_num,2)-acc_offset(1))*acc_scal(2);
        acc(1:last_num,3) = (acc(1:last_num,3)-acc_offset(3))*acc_scal(3);
    end
    %% Magnetometer data correction
    if last_num == 10
        %% Ellipsoid fitting
%         [Center,Scale_axis] = fit_elliposoid9(mag);
%         mag_offset(1) = Center(1);
%         mag_offset(2) = Center(2);
%         mag_offset(3) = Center(3);
%         mag_scal(1) = 1;%recipNorm * Scale_axis(1);
%         mag_scal(2) =  Scale_axis(1)/Scale_axis(2);
%         mag_scal(3) =  Scale_axis(1)/Scale_axis(3);
       %% Maximum/Minimum Fitting
%         [Xmax,~]=max(mag(:,1));
%         [Xmin,~]=min(mag(:,1));
%         [Ymax,~]=max(mag(:,2));
%         [Ymin,~]=min(mag(:,2));
%         [Zmax,~]=max(mag(:,3));
%         [Zmin,~]=min(mag(:,3));
%         mag_offset(1) = (Xmax+Xmin)/2;
%         mag_offset(2) = (Ymax+Ymin)/2;
%         mag_offset(3) = (Zmax+Zmin)/2;
%         mag_scal(1) = 1;
%         mag_scal(2) = ((Xmax-Xmin)/(Ymax-Ymin));
%         mag_scal(3) = ((Xmax-Xmin)/(Zmax-Zmin));   
        %%  function calculated Fitting
        %20Hz
%         mag_offset = [25.006799396836282,38.734546820244180,-5.582208482360802];
%         mag_scal = [1,1.078428963317943,0.952871831718862];
        %100HZ
        mag_offset = [93.084022482510900,1.573560237465101e+02,-39.433303431889700];%[99.755721966979410,1.573614143920282e+02,-36.165910391776560];
        mag_scal = [1,1.076469733311434,0.957304748470665];%[1,1.109617153356717,1];
        %% Magnetometer data correction
        mag(1:last_num,1) = (mag(1:last_num,1)-mag_offset(1))*mag_scal(1);
        mag(1:last_num,2) = (mag(1:last_num,2)-mag_offset(2))*mag_scal(2);
        mag(1:last_num,3) = (mag(1:last_num,3)-mag_offset(3))*mag_scal(3);
    end
    %% Normal posture data correction
    if last_num > 10 %%The calculated angle value after outputting the corrected data
%         Data correction
        gyro(last_num,1:3) = (gyro(last_num,1:3)-gyro_offset(1:3)).*gyro_scal(1:3);
        acc(last_num,1:3)  = (acc(last_num,1:3)-acc_offset(1:3)).* acc_scal(1:3);
        mag(last_num,1:3)  = (mag(last_num,1:3)-mag_offset(1:3)).* mag_scal(1:3);
        NonlinearSO3AHRSupdate(gyro(last_num,1), gyro(last_num,2), gyro(last_num,3),...
							-acc(last_num,1), -acc(last_num,2), -acc(last_num,3),...
							mag(last_num,1), -mag(last_num,2), mag(last_num,3),...
							1.0,... 
							0.05,...
							dt);
%         NonlinearSO3AHRSupdate(0, 0,0,...
% 							0.1270, 0.5288, 0.8179,...
% 							-182, -207, -104,...
% 							1.0,...   
% 							0.05,...
% 							dt);
% 		Convert q->R, This R converts inertial frame to body frame. 
		Rot_matrix(1,1) = q0q0 + q1q1 - q2q2 - q3q3;
		Rot_matrix(1,2) = 2.0 * (q1q2 + q0q3);	
		Rot_matrix(1,3) = 2.0 * (q1q3 - q0q2);	
		Rot_matrix(2,1) = 2.0 * (q1q2 - q0q3);	
		Rot_matrix(2,2) = q0q0 - q1q1 + q2q2 - q3q3;
		Rot_matrix(2,3) = 2.0 * (q2q3 + q0q1);	
		Rot_matrix(3,1) = 2.0 * (q1q3 + q0q2);	
		Rot_matrix(3,2) = 2.0 * (q2q3 - q0q1);	
		Rot_matrix(3,3) = q0q0 - q1q1 - q2q2 + q3q3;
        % Obtain the Euler angle radian system according to the rotation matrix
		euler(1) = atan2(Rot_matrix(2,3), Rot_matrix(3,3));	%Roll
		euler(2) = -asin(Rot_matrix(1,3));				    %Pitch
		euler(3) = atan2(Rot_matrix(1,2), Rot_matrix(1,1));	%Yaw
        % Euler angle value
        roll  = euler(1)*57.2957795;
        pitch = euler(2)*57.2957795;
        yaw   = euler(3)*57.2957795;
        Angle = atan2(-mag(last_num,2),mag(last_num,1))*57.2957795;
       % Model rotation
        rotate(model,[1 0 0],mod(roll-last_roll,360));
        rotate(model,[0 1 0],mod(pitch-last_pitch,360));
        rotate(model,[0 0 1],mod(yaw-last_yaw,360));
       last_roll = roll;
       last_pitch = pitch;
       last_yaw = yaw;
       fprintf('num = %d last_num= %d roll = %f pitch = %f yaw = %f angel = %f \n',num,last_num,roll,pitch,yaw,Angle);
       data_error_counter = data_error_counter+1;
       data_error_t = [data_error_t;data_error_counter];
       data_error = [data_error;roll,pitch,yaw];
    end
    set(roll_error,'XData',data_error_t(:),'YData',data_error(:,1));
    set(pitch_error,'XData',data_error_t(:),'YData',data_error(:,2));
    set(yaw_error,'XData',data_error_t(:),'YData',data_error(:,3));
    drawnow
end