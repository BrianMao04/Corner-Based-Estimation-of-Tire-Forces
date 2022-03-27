%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Brian Mao
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Vehicle Parameter Constants
R_e = 0.33;
I_w = 1.68;
m = 2270;
Lf = 1.42;
Lr = 1.43;
Iz = 4650;
Trf = 1.62;
Trr = 1.56;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Loading Experimental Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Wheel Torques
fig = openfig('Fig8_WheelTorques.fig');
dataObjs = findobj(fig,'-property','XData');
X_Values = dataObjs(1).XData; %X_Axis values are the same across all plots

dataObjs = findobj(fig,'-property','YData');
Torque_Total_Rear_Right = dataObjs(1).YData; %Teal Line (FB_TR_R);
Torque_Total_Rear_Left = dataObjs(2).YData; %Red Line (FB_TR_L);
Torque_Total_Front_Right = dataObjs(3).YData; %Green Line (FB_TF_R);
Torque_Total_Front_Left = dataObjs(4).YData; %Blue Line (FB_TF_L);

%Wheel Speeds
fig = openfig('Fig8_Accelerations_WheelSpeed.fig');
dataObjs = findobj(fig,'-property','YData');
WheelSpeed_Rear_Right = dataObjs(1).YData;  %rR
WheelSpeed_Rear_Left = dataObjs(2).YData;   %rL
WheelSpeed_Front_Right = dataObjs(3).YData; %fR
WheelSpeed_Front_Left = dataObjs(4).YData;  %fL

%Lateral Force Measurements
fig = openfig('Fig8_FyFz.fig');
dataObjs = findobj(fig,'-property','YData');
Lat_Fy_Estimate = dataObjs(3).YData; 
Lat_Fy_measurement = dataObjs(4).YData; 

%Steering Angle
fig = openfig('Fig8_SteeringWheelAngle.fig');
dataObjs = findobj(fig,'-property','YData');
Delta_angle_driver = dataObjs(1).YData; 
Delta_angle_tire = dataObjs(1).YData/17.2;
%Note that the steering wheel angle data was measured from the driver hand.
%The measurements are divided by 17.2 to approximate the steering angle at the tires.

%Yaw Rate and Vehicle Accelerations
fig = openfig('Fig8_Accelerations_YawRate');
dataObjs = findobj(fig,'-property','YData');
Yaw_Rate = dataObjs(1).YData;  
Acceleration_y = dataObjs(2).YData; 
Acceleration_x = dataObjs(3).YData;

%Close Plots After Extracting Relevant Data
close all; 

%%%%%%%%%%%%%%%%%%%%%%
%Initial Conditions
%%%%%%%%%%%%%%%%%%%%%%
Yaw_Rate_IC = Yaw_Rate(1);
Acceleration_y_IC = Acceleration_y(1);
Acceleration_x_IC = Acceleration_x(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Wheel Acceleration Calculation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WheelAcceleration_Rear_Right = gradient(WheelSpeed_Rear_Right);
WheelAcceleration_Rear_Left = gradient(WheelSpeed_Rear_Left);
WheelAcceleration_Front_Right = gradient(WheelSpeed_Front_Right);
WheelAcceleration_Front_Left = gradient(WheelSpeed_Front_Left);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Longitudinal Force Calculation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Timesteps
time = X_Values;

%Longitudinal Force Estimates
Fx_Rear_Right_Estimate = zeros(1,length(time));
Fx_Rear_Left_Estimate = zeros(1,length(time));
Fx_Front_Right_Estimate = zeros(1,length(time));
Fx_Front_Left_Estimate = zeros(1,length(time));

for t=1:length(time) 
    Fx_Rear_Right_Estimate(t) = (Torque_Total_Rear_Right(t)-I_w*WheelAcceleration_Rear_Right(t))/R_e;
    Fx_Rear_Left_Estimate(t) = (Torque_Total_Rear_Left(t)-I_w*WheelAcceleration_Rear_Left(t))/R_e;
    Fx_Front_Right_Estimate(t) = (Torque_Total_Front_Right(t)-I_w*WheelAcceleration_Front_Right(t))/R_e;
    Fx_Front_Left_Estimate(t) = (Torque_Total_Front_Left(t)-I_w*WheelAcceleration_Front_Left(t))/R_e;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Lateral Force Estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Noise (Time-Invariant)
Q_bar = (0.13^2)*eye(3,3);
R_bar = (0.012^2)*eye(3,3);

Omega_bar = [0.13; 0.13; 0.13]; 
Gamma_bar = [0.012; 0.012; 0.012]; 

%State at Each Time Step
X_vector = zeros(3,length(time)); 

%Sigma Point Properties
N = 3; 
Tau_bar = sqrt(3);

%Sigma Points (Stored across 3 vectors with each corresponding to a different component)
sigma_points_1 = zeros(1,2*N+1);
sigma_points_2 = zeros(1,2*N+1);
sigma_points_3 = zeros(1,2*N+1);

%Weighting Parameters
W_ic = 3/2; %For Covariance Calculations
W_oc = 2;
W_im = 3/2; %For Mean Calculations
W_om = 0;


%%%%%%%%%%%%%%%%
%Initialization
%%%%%%%%%%%%%%%%
X_vector(:,1)= [1371.4; 1371.4; 500];    
Pk = 20*eye(3); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Main Iteration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=2:length(time) 
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    %Prediction Step
    %%%%%%%%%%%%%%%%%%%%%%%%
    %Estimated Mean Update
    x_hat_mk = X_vector(:,t-1);
    
    %State Covariance Matrix 
    P_mk = Pk + Q_bar; %Including additive process noise
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    %Sigma Point Generation 
    %%%%%%%%%%%%%%%%%%%%%%%%
    %Centered Sigma Point 
    sigma_points_1(1) = x_hat_mk(1);
    sigma_points_2(1) = x_hat_mk(2);
    sigma_points_3(1) = x_hat_mk(3);
    
    Pk_chol = chol(P_mk);
    
    %Right Sigma Points
    for i=2:4 
        sigma_points_1(i) = x_hat_mk(1) + Tau_bar*Pk_chol(1,i-1); 
        sigma_points_2(i) = x_hat_mk(2)+ Tau_bar*Pk_chol(2,i-1); 
        sigma_points_3(i) = x_hat_mk(3)+ Tau_bar*Pk_chol(3,i-1);
    end
    
    %Left Sigma Points
    for i=5:7 
        sigma_points_1(i) = x_hat_mk(1) - Tau_bar*Pk_chol(1,i-4); 
        sigma_points_2(i) = x_hat_mk(2)- Tau_bar*Pk_chol(2,i-4); 
        sigma_points_3(i) = x_hat_mk(3)- Tau_bar*Pk_chol(3,i-4);
    end
    
    %%%%%%%%%%%%%%%%%%%%
    %Correction Step
    %%%%%%%%%%%%%%%%%%%%
    %Transformed Outputs/Measurements
    Sigma_Points = zeros(3,2*N+1);
    Y_Transformed = zeros(3,2*N+1);
    
    %Sigma Point Propagation
    for i=1:length(sigma_points_1)
        sigma_point_Y_Vector = [sigma_points_1(i); sigma_points_2(i); sigma_points_3(i)];
        Sigma_Points(:,i) = sigma_point_Y_Vector;
        Y_Transformed(:,i) = yk(Delta_angle_tire(t),Fx_Front_Right_Estimate(t),Fx_Front_Left_Estimate(t),Fx_Rear_Right_Estimate(t),Fx_Rear_Left_Estimate(t), sigma_point_Y_Vector , Gamma_bar);
    end
    
    %Output Mean
    y_hat_mk_1 = W_om*Y_Transformed(1,1);
    y_hat_mk_2 = W_om*Y_Transformed(2,1);
    y_hat_mk_3 = W_om*Y_Transformed(3,1);
    
    for i=2:7
        y_hat_mk_1 = y_hat_mk_1 + W_im*Y_Transformed(1,i);
        y_hat_mk_2 = y_hat_mk_2 + W_im*Y_Transformed(2,i);
        y_hat_mk_3 = y_hat_mk_3 + W_im*Y_Transformed(3,i);
    end
    y_hat_mk = [y_hat_mk_1 ;y_hat_mk_2 ;y_hat_mk_3];
    
 
    %Measurement Covariance Matrix
    P_yk_yk = W_oc*(Y_Transformed(:,1)-y_hat_mk)*(Y_Transformed(:,1)-y_hat_mk).';
    for i=2:7
        P_yk_yk = P_yk_yk + W_ic*(Y_Transformed(:,i)-y_hat_mk)*(Y_Transformed(:,i)-y_hat_mk).';
    end
    P_yk_yk = P_yk_yk + R_bar; %Including additive measurement noise
 
    
    %Cross-Covariance Matrix 
    P_xk_yk = W_oc*(Sigma_Points(:,1)-x_hat_mk)*(Y_Transformed(:,1)-y_hat_mk).';
    for j=2:7
        P_xk_yk = P_xk_yk + W_ic*(Sigma_Points(:,j)-x_hat_mk)*(Y_Transformed(:,j)-y_hat_mk).';
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %State and Covariance Update 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Kalman Gain
    Kk = P_xk_yk*inv(P_yk_yk);
    
    %Updated Measurements
    y_actual = [Acceleration_x(t); Acceleration_y(t); Yaw_Rate(t)];
    
    %State and Covariance Update
    X_vector(:,t) = x_hat_mk + (Kk*(y_actual-y_hat_mk)); 
    Pk = P_mk - (Kk*P_yk_yk*Kk.'); 
    
    %Note: The following lines of code is to account for numerical roundoff errors within Matlab. 
    %Very small entries in the cross covaraince matrix are rounded to zero.
    %Otherwise, the resulting covariance matrix may not be postive definite,
    %and hence a cholesky decompostion cannot be performed at the subsequent time step.
    try
        chol(Pk + Q_bar)
    catch
        P_xk_yk(abs(P_xk_yk)<0.001)=0;
        Kk = P_xk_yk*inv(P_yk_yk); 
        Pk = P_mk - (Kk*P_yk_yk*Kk.');
        try
            chol(Pk + Q_bar)
        catch
            P_xk_yk(abs(P_xk_yk)<0.2)=0;
            Kk = P_xk_yk*inv(P_yk_yk);
            Pk = P_mk - (Kk*P_yk_yk*Kk.');
        end
    end
end

%%%%%%%%%%%%%%
%Plotting
%%%%%%%%%%%%%%
Fy_Front_Left = 0.25*(X_vector(1,:)-X_vector(3,:));
Fy_Front_Left = Fy_Front_Left/1000; %Convert from N to kN

subplot(2,1,1)
hold on
plot(X_Values,Fy_Front_Left)
plot(X_Values,Lat_Fy_measurement)
xlabel('t (Seconds)')
ylabel('Fy (kN)')
xlim([0 10])
ylim([-4 8])
title('Lateral Force Estimation')
legend('Estimated F_y From UKF', 'Experimental F_y Measurement')
box on
hold off
    