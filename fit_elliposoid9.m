function[ Center,Scale_axis] = fit_elliposoid9(data)
    % Description: Ellipsoid fitting algorithm
    % input data is n*3,  n points of the ellipsoid surface
    % Least Square Method
    %  a(1)x^2+a(2)y^2+a(3)z^2+a(4)xy+a(5)xz+a(6)yz+a(7)x+a(8)y+a(9)z=1
    %% Require Data
    x=data(:,1);
    y=data(:,2);
    z=data(:,3);
    D=[x.*x y.*y z.*z x.*y x.*z y.*z x y z ];
    a=inv(D'*D)*D'*ones(size(x));
    M=[a(1)    a(4)/2  a(5)/2;...
       a(4)/2  a(2)    a(6)/2;...
       a(5)/2  a(6)/2  a(3)]; 
    Center=-1/2*[a(7),a(8),a(9)]*inv(M);  
    SS=Center*M*Center'+1;
    [U,V]=eig(M);
    [~,n1]=max(abs(U(:,1)));
    [~,n2]=max(abs(U(:,2)));
    [~,n3]=max(abs(U(:,3)));
    lambda(1)=V(n1,n1);
    lambda(2)=V(n2,n2);
    lambda(3)=V(n3,n3);
    Scale_axis=[sqrt(SS/lambda(1)),sqrt(SS/lambda(2)),sqrt(SS/lambda(3))];        
    %center
    %Center=round(Center);
    %Scale_axis=round(Scale_axis);
    Center,Scale_axis
    figure;
    plot3(x,y,z,'b.',Center(1),Center(2),Center(3),'ro');
    axis equal
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Fit--elliposoid');
end