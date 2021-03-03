function NonlinearSO3AHRSinit(ax, ay,  az, mx, my,  mz)
% Using accelerometer, sense the gravity vector.
% Using magnetometer, sense yaw.
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
    if ~((ax == 0.0) && (ay == 0.0) && (az == 0.0))
        initialRoll = atan2(-ay,-az);
        initialPitch = atan2(ax,-az);
        cosRoll = cos(initialRoll);   
        sinRoll = sin(initialRoll);   
        cosPitch = cos(initialPitch); 
        sinPitch = sin(initialPitch); 
        magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch; 
        magY = my * cosRoll - mz * sinRoll;
        initialHdg = atan2(-magY, magX);
        cosRoll = cos(initialRoll * 0.5);
        sinRoll = sin(initialRoll * 0.5);
        cosPitch = cos(initialPitch * 0.5);
        sinPitch = sin(initialPitch * 0.5);
        cosHeading = cos(initialHdg * 0.5);
        sinHeading = sin(initialHdg * 0.5);
        q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    else
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
    end
    % auxillary variables to reduce number of repeated operations, for 1st pass
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