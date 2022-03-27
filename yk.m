%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Measurement Model Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = yk(Delta_angle_tire,Fx_Front_Right_Estimate,Fx_Front_Left_Estimate,Fx_Rear_Right_Estimate,Fx_Rear_Left_Estimate, X_vector, Gamma_bar) 
%Vehicle Parameters
m = 2270;
Lf = 1.42;
Lr = 1.43;
Iz = 4650;
Trf = 1.62;
Trr = 1.56;

%Matricies
C_bar = zeros(3,3);
u_bar = zeros(3,1);

%Update C_bar
C_bar(1,1) = -(1/m)*sin(Delta_angle_tire);
C_bar(2,1) = (1/m)*cos(Delta_angle_tire);
C_bar(2,2) = 1/m;
C_bar(3,1) = (1/Iz)*cos(Delta_angle_tire)*Lf;
C_bar(3,2) = -(1/Iz)*Lr;
C_bar(3,3) = (1/(2*Iz))*sin(Delta_angle_tire)*Trf;

%Update u_bar
Fxf = Fx_Front_Right_Estimate + Fx_Front_Left_Estimate;
Fxr = Fx_Rear_Right_Estimate + Fx_Rear_Left_Estimate;
Fxf_bar = Fx_Front_Right_Estimate - Fx_Front_Left_Estimate;
Fxr_bar = Fx_Rear_Right_Estimate - Fx_Rear_Left_Estimate;

u_bar(1,1) = (1/m) *(Fxf*cos(Delta_angle_tire) + Fxr);
u_bar(2,1) = (1/m) * (Fxf*sin(Delta_angle_tire));
u_bar(3,1)= (1/Iz)*((Fxf*sin(Delta_angle_tire)*Lf) + (Fxf_bar*cos(Delta_angle_tire)*0.5*Trf) + (Fxr_bar*0.5*Trr));   

output = C_bar*X_vector + u_bar + Gamma_bar; 
end