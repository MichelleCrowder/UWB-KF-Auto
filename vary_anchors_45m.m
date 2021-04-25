%UWB Project

%In this code, you can change the positions of the anchors or the radius of
%the path in order to observe the LS RMSE and Kalman RMSE from the model

clear all
close all


m = 6; %change this to the number of anchors you'd like
%m can also be a vector to run multiple trials in succession.
%sweep will track the LS and Kalman RMSE over the different configuration
%ran
sweep = zeros(3, length(m));

%preset anchors - you may modify these positions, or keep them.
A = [6 6; 6 7.9; 10.6 6; 10.6 7.9; 8.3 6; 8.3 7.9;... %Anchors 1-6
    7.15 6.95; 9.45 6.95; 9.45 7.9; 9.45 6; 7.15 7.9; 7.15 6; ... %Anchors 7-12
    6 6.95; 10.6 6.95; 8.3 6.95].'; %Anchors 13-15

%if length(m) > 1 then the for loop will run for each anchor configuration
for p = 1: length(m)
%Sampling every 0.25s
dt = 0.25;
v = 1; %velocity
%Each measurement is modeled by a gaussian random variable around the true
%range with a variance of 0.5 m^2
k = 0:dt:33; %k is the vector of time steps
r = 4.5;  %radius of the path from the center of the vehicle in meters (This can be changed if desired)
c = [8.3, 6.95]; %center of the vehicle
path(:,1) = r * cos(k/r) + c(1); %x value of path
path(:,2) = r * sin(k/r) + c(2); %y value of path

hold on
p1 = plot(path(:,1), path(:,2));  %path of the tag
Aall = (A(:,1:m(p))).';  %matrix of anchors
plot(Aall(:,1), Aall(:,2),'k*')
rectangle('Position', [6 6 4.6 1.9], 'EdgeColor', 'b') %vehicle outline

Nm = length(Aall); %Number of measurements, or anchors

%kalman filter and system set up

Fk = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
Gk = [1, 0, 0, 0; 0, 1, 0, 0];
B = [0;0;0;0];
D = [0;0];

eta = 1.75;
R = [eta, 0; 0, eta]; %The covariance of the measurement noise
Q = [0, 0, 0,0; 0,0,0,0; 0,0,dt^2, 0; 0,0,0,dt^2]; %Covariance of the state

iter = 1; %number of times to run.
SumKError = 0;
SumLSError = 0;
K_Error = zeros(length(k),iter);

%For loop of iterations
for n = 1:iter

    zdis = zeros(Nm, length(k));
    xydata = zeros(Nm, length(k));
    var = .5;
    
    %For loop to create noisy measurement, zdis
    for i = 1: length(k)
        for j = 1:Nm %number of measurements
            noise = normrnd(0,sqrt(var));
            zdis(j,i) = sqrt((path(i,1) - Aall(j,1))^2 + (path(i,2) - Aall(j,2))^2) + noise;
        end
    end
    
    %least squares algorithm
    fun = @(xy,Aall)(sqrt((xy(:,1) - Aall(:,1)).^2 + (xy(:,2) - Aall(:,2)).^2));
    x0 = [9,7]; %initial estimate is approximately center of the vehicle
    
    LSestimate = zeros(length(k),2);
    kalmest = zeros(length(k),2);
    
    LS_distsq = zeros(length(k),1);
    K_distsq = zeros(length(k),1);
    
    %Kalman filter
    for i = 1: length(k)
        
        %theta is the x and y estimate from the LS algorithm
        theta = lsqcurvefit(fun,x0,Aall,(zdis(:,i)));
        zk = [theta(1); theta(2)]; %the estimate from the LS algorithm is zk
        %set up initial values for k = 1
        if i == 1
            muk1k1 = [theta(1); theta(2); 0;0];
            v = [1,1,1,1];
            Pk1k1 = diag(v);
        end
        mukk1 = Fk*muk1k1;
        Pkk1 = Q + Fk*Pk1k1*Fk.';
        Kk = Pkk1 * Gk.' * inv(Gk*Pkk1*Gk.' + R); %Kalman gain matrix
        
        mukk = mukk1 + Kk *(zk - Gk*mukk1); %state estimate (position and velocity)
        Pkk = Pkk1 - Kk*Gk*Pkk1;
        
        kalmest(i,1) = mukk(1); %x value estimate
        kalmest(i,2) = mukk(2); %y value estimate
        x0 = [mukk(1), mukk(2)];
        LSestimate(i,:) = theta;
        
        %We only need to plot one iteration
        if n==1
            p3 = plot(theta(1), theta(2), 'rx');
        end
        %update the mu and P vectors
        muk1k1 = mukk;
        Pk1k1 = Pkk;
        
        %distance between LS estimate and path squared
        LS_distsq(i) = (theta(1) - path(i,1))^2 + (theta(2) - path(i,2))^2;
        K_distsq(i) = (mukk(1) - path(i,1))^2 +(mukk(2) - path(i,2))^2;
        
    end
    
    LSdis = sum(LS_distsq);
    Kaldis = sum(K_distsq);
    K_Error(:,n) = K_distsq;
    
    LSrmse = sqrt(LSdis/length(path));
    Kalrmse = sqrt(Kaldis/length(path));
    
    SumLSError = LSrmse + SumLSError;
    SumKError = Kalrmse + SumKError; %This is summing the error over the iterations
    
    %plotting
    if n == 1
        p4 = plot(kalmest(:,1),kalmest(:,2), 'g');
        
        xlabel('x(m)')
        ylabel('y(m)')
        for i = 1:Nm
            txt = [' ',num2str(i)];
            text(Aall(i,1), Aall(i,2), txt)
        end
        legend([p1, p3, p4], 'Target Path', 'LS Estimates', 'Kalman Estimates')
        axis([0 16 -1 15])
    end
    
end %This is the end of the iterations

%Mean least square error
avgLSerr = SumLSError/iter %for the LS algorithm
avgKerr = SumKError/iter %for the Kalman filter
Kerr_ang = sum(K_Error,2)./iter; %Average of the error at each angle.
ang = k/r*180/pi;

%plot the error at each angle. The origin is at the center of the vehicle.
hold off
figure(2)
plot((k./r*180/pi),Kerr_ang)
xlabel('Angle (Degrees)'); ylabel('Squared Error (m^2)'); title('Squared Error vs. angle'); 
axis([40 450 0.1 .5])

sweep(:,p) = [m(p), avgLSerr, avgKerr];

end %this is the end of the parameter sweep.

