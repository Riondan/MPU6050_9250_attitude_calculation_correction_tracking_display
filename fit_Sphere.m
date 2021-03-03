function[ Center,Scale_axis] = fit_Sphere(data)
    % Description: Spherical fitting algorithm
    % input data is n*3,  n points of the ellipsoid surface
    % Least Square Method
    x=data(:,1);
    y=data(:,2);
    z=data(:,3);
    num_points = length(x);
    x_avr = sum(x)/num_points;
    y_avr = sum(y)/num_points;
    z_avr = sum(z)/num_points;
    xx_avr = sum(x.*x)/num_points;
    yy_avr = sum(y.*y)/num_points;
    zz_avr = sum(z.*z)/num_points;
    xy_avr = sum(x.*y)/num_points;
    xz_avr = sum(x.*z)/num_points;
    yz_avr = sum(y.*z)/num_points;

    xxx_avr = sum(x.*x.*x)/num_points;
    xxy_avr = sum(x.*x.*y)/num_points;
    xxz_avr = sum(x.*x.*z)/num_points;
    xyy_avr = sum(x.*y.*y)/num_points;
    xzz_avr = sum(x.*z.*z)/num_points;
    yyy_avr = sum(y.*y.*y)/num_points;
    yyz_avr = sum(y.*y.*z)/num_points;
    yzz_avr = sum(y.*z.*z)/num_points;
    zzz_avr = sum(z.*z.*z)/num_points;

    A = [xx_avr - x_avr*x_avr,xy_avr - x_avr*y_avr,xz_avr - x_avr*z_avr;
         xy_avr - x_avr*y_avr,yy_avr - y_avr*y_avr,yz_avr - y_avr*z_avr;
         xz_avr - x_avr*z_avr,yz_avr - y_avr*z_avr,zz_avr - z_avr*z_avr];
    b = [xxx_avr - x_avr*xx_avr + xyy_avr - x_avr*yy_avr + xzz_avr - x_avr*zz_avr;
         xxy_avr - y_avr*xx_avr + yyy_avr - y_avr*yy_avr + yzz_avr - y_avr*zz_avr;
         xxz_avr - z_avr*xx_avr + yyz_avr - z_avr*yy_avr + zzz_avr - z_avr*zz_avr];
    b = b/2;

    resoult = inv(A)*b;

    x00 = resoult(1);     
    y00 = resoult(2);     
    z00 = resoult(3);    
    r = sqrt(xx_avr-2*x00*x_avr+x00*x00 + yy_avr-2*y00*y_avr+y00*y00 + zz_avr-2*z00*z_avr+z00*z00); 

    Center = [x00 y00 z00];
    Scale_axis = [r r r];

    Center,Scale_axis

    [u,v,w]=sphere;
    hold on;plot3(u*r+x00,v*r+y00,w*r+z00);

    figure;
    plot3(x,y,z,'b.',Center(1),Center(2),Center(3),'ro');
    hold on;plot3(u*r+x00,v*r+y00,w*r+z00);
    axis equal
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Fit--Sphere');
end