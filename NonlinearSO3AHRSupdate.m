function NonlinearSO3AHRSupdate(gx, gy, gz, ax, ay,  az, mx,  my, mz, twoKp, twoKi,dt)
% Description: Attitude calculation fusion, Crazepony and core algorithm
% The Mahony complementary filtering algorithm is used, and the Kalman filtering algorithm is not used
% The modified algorithm is a direct reference to the algorithm of the pixhawk flight control, you can see the source on Github
% https://github.com/hsteinhaus/PX4Firmware/blob/master/src/modules/attitude_estimator_so3/attitude_estimator_so3_main.cpp
% Input parameters: 
% gx,gy,gz:Gyro data
% ax,ay,az:Accelerometer data
% mx,my,mz:Magnetometer data
% twoKp,twoKi:Proportional coefficient, integral coefficient
% dt:The step length of the integration, that is, the time interval
% quaternion of sensor frame relative to auxiliary frame
    global q0;
    global q1;
    global q2;
    global q3;
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
    global bFilterInit;
    %quaternion of sensor frame relative to auxiliary frame 
    global dq0;
    global dq1;
    global dq2;
    global dq3;
    global gyro_bias;
    halfex = 0;
    halfey = 0;
    halfez = 0; 
% 	Make filter converge to initial solution faster 
% 	This function assumes you are in static position. 
    if  bFilterInit == 0
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
		bFilterInit = 1;
    end
    if ~(mx == 0.0 && my == 0.0 && mz == 0.0)
		%Normalise magnetometer measurement
    	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    	mx = mx * recipNorm;
    	my = my * recipNorm;
    	mz = mz * recipNorm;
    	%Reference direction of Earth's magnetic field
    	hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    	hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0 * mx * (q1q3 - q0q2) + 2.0 * my * (q2q3 + q0q1) + 2.0 * mz * (0.5 - q1q1 - q2q2);
    	bx = sqrt(hx * hx + hy * hy);
    	bz = hz;    
    	%Estimated direction of magnetic field
    	halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    	halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);
    	% Error is sum of cross product between estimated direction and measured direction of field vectors 
        halfex = halfex + (my * halfwz - mz * halfwy);
        halfey = halfey + (mz * halfwx - mx * halfwz);
        halfez = halfez + (mx * halfwy - my * halfwx);
    end
%   Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if ~(ax == 0.0 && ay == 0.0 && az == 0.0) 
% 		Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax = ax * recipNorm;
		ay = ay * recipNorm;
		az = az * recipNorm;
% 		Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5 + q3q3;
% 		Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = halfex + ay * halfvz - az * halfvy;
		halfey = halfey + az * halfvx - ax * halfvz;
		halfez = halfez + ax * halfvy - ay * halfvx;
    end  
% 	Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if halfex ~= 0.0 && halfey ~= 0.0 && halfez ~= 0.0  
% 		Compute and apply integral feedback if enabled
        if twoKi > 0.0
			gyro_bias(1) = gyro_bias(1) + twoKi * halfex * dt;
			gyro_bias(2) = gyro_bias(2) + twoKi * halfey * dt;
			gyro_bias(3) = gyro_bias(3) + twoKi * halfez * dt;
%           apply integral feedback
			gx = gx + gyro_bias(1);
			gy = gy + gyro_bias(2);
			gz = gz + gyro_bias(3);
        else
			gyro_bias(1) = 0.0;	
			gyro_bias(2) = 0.0;
			gyro_bias(3) = 0.0;
        end
% 		Apply proportional feedback
		gx = gx + twoKp * halfex;
		gy = gy + twoKp * halfey;
 		gz = gz + twoKp * halfez;
    end
% 	Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
% 	q_k = q_{k-1} + dt*\dot{q}
% 	dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5*(q0 * gz + q1 * gy - q2 * gx); 
	q0 = q0 + dt*dq0;
	q1 = q1 + dt*dq1;
	q2 = q2 + dt*dq2;
	q3 = q3 + dt*dq3;
% 	Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * recipNorm;
	q1 = q1 * recipNorm;
	q2 = q2 * recipNorm;
	q3 = q3 * recipNorm;
% 	Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;   
end