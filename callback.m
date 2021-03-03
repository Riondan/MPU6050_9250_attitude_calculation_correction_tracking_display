function callback(s, BytesAvailable,p,model,xy_point,xz_point,yz_point)
    global num;
    global acc;
    global gyro;
    global mag;
    out = fscanf(s);
    data = str2num(out);
    %% 分解数据
    if length(data)==9
        if ~(abs(data(1,1))>30000 || abs(data(1,2))>30000 || abs(data(1,3))>30000 ...
             || abs(data(1,4))>30000 || abs(data(1,5))>30000 || abs(data(1,6))>30000 ...
             || abs(data(1,7))>1000 || abs(data(1,8))>1000 || abs(data(1,9))>1000 )
                acc  = [acc ;-data(1:3)/32768*4*9.7833];%8g
                gyro = [gyro;data(4:6)/32768*1000/57.2957795];
                mag  = [mag ;data(7:9)*0.6];
                num = num +1 ;
                
         end
    end
end