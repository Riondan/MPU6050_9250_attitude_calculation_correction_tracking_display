function[ Center,Scale_axis] = fit_maxmin(data)
    % Description: Spherical fitting algorithm (maximum and minimum)
    % input data is n*3,  n points of the ellipsoid surface
    % Least Square Method
    %% Require Data
    x=data(:,1);
    y=data(:,2);
    z=data(:,3);
    [Xmax,~]=max(x);
    [Xmin,~]=min(x);
    [Ymax,~]=max(y);
    [Ymin,~]=min(y);
    [Zmax,~]=max(z);
    [Zmin,~]=min(z);
    Center(1) = (Xmax+Xmin)/2;
    Center(2) = (Ymax+Ymin)/2;
    Center(3) = (Zmax+Zmin)/2;
    Scale_axis(1) = (Xmax-Xmin);
    Scale_axis(2) = (Ymax-Ymin);
    Scale_axis(3) = (Zmax-Zmin);      
    Center,Scale_axis
    figure;
    plot3(x,y,z,'b.',Center(1),Center(2),Center(3),'ro');
    hold on;
    axis equal
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Fit--MaxMin');
end