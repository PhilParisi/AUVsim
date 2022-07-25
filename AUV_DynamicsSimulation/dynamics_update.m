function AUV = dynamic_update(AUV, TAU_P,dt,flag)

%%%%AUV dyanmics%%%%%

%%resotring force matrix
G = [ (AUV.B - AUV.W)* sin(AUV.rpy(2)); ...
       (AUV.W-AUV.B) * cos(AUV.rpy(2)) * sin(AUV.rpy(1));...
       (AUV.W - AUV.B) * cos(AUV.rpy(2))* cos(AUV.rpy(1));...
       (AUV.YG*AUV.W-AUV.YB*AUV.B) * cos(AUV.rpy(2)) * cos(AUV.rpy(1)) - (-AUV.ZB*AUV.B+AUV.ZG*AUV.W)*cos(AUV.rpy(2))*sin(AUV.rpy(1));...
      -(AUV.ZG*AUV.W - AUV.ZB*AUV.B) * sin(AUV.rpy(2)) + (AUV.XB*AUV.B-AUV.XG*AUV.W) * cos(AUV.rpy(2)) * cos(AUV.rpy(1));...
       (AUV.XG*AUV.W-AUV.XB*AUV.B) * cos(AUV.rpy(2)) * sin(AUV.rpy(1)) + (AUV.YG*AUV.W-AUV.YB*AUV.B) *sin(AUV.rpy(2))];
          
%%Added inertia
CAV11= 0*eye(3);  %%zero matrix
CAV12 = [0                      AUV.Z_wdot*AUV.uvw(3)     -AUV.Y_vdot*AUV.uvw(2);...
         -AUV.Z_wdot*AUV.uvw(3)                      0      AUV.X_udot*AUV.uvw(1);...
        AUV.Y_vdot*AUV.uvw(2)   -AUV.X_udot*AUV.uvw(1)                        0];
CAV21 = CAV12;
     
CAV22 = [0                       AUV.N_rdot*AUV.pqr(3)     -AUV.M_qdot*AUV.pqr(2);...
          -AUV.N_rdot*AUV.pqr(3)                      0       AUV.K_pdot*AUV.pqr(1);...
         AUV.M_qdot*AUV.pqr(2)  -AUV.K_pdot*AUV.pqr(1)                          0];


CAV = [CAV11 CAV12; CAV21 CAV22];

%%%Damping
DV=[AUV.X_u2*abs(AUV.uvw(1));...
   AUV.Y_v2*abs(AUV.uvw(2));...
   AUV.Z_w2*abs(AUV.uvw(3));...
   AUV.K_p2*abs(AUV.pqr(1));...
   AUV.M_q2*abs(AUV.pqr(2));...
   AUV.N_r2*abs(AUV.pqr(3))];

%%rigid body inertia
CRB11 = 0*eye(3);
CRB12 = [AUV.m*(AUV.YG*AUV.pqr(2)+AUV.ZG*AUV.pqr(3))    -AUV.m*(AUV.XG*AUV.pqr(2)-AUV.uvw(3))         -AUV.m*(AUV.XG*AUV.pqr(3)+AUV.uvw(2));...
        -AUV.m*(AUV.YG*AUV.pqr(1)+AUV.uvw(3))           AUV.m*(AUV.ZG*AUV.pqr(3)+AUV.XG*AUV.pqr(1))  -AUV.m*(AUV.YG*AUV.pqr(3)-AUV.uvw(1));...
        -AUV.m*(AUV.ZG*AUV.pqr(1)-AUV.uvw(2))           -AUV.m*(AUV.ZG*AUV.pqr(2)+AUV.uvw(1))         AUV.m*(AUV.XG*AUV.pqr(1)+AUV.YG*AUV.pqr(2))];

CRB12 = [AUV.m*(AUV.YG*AUV.pqr(2)+AUV.ZG*AUV.pqr(3))    -AUV.m*(AUV.XG*AUV.pqr(2)-AUV.uvw(3))         -AUV.m*(AUV.XG*AUV.pqr(3)+AUV.uvw(2));...
        -AUV.m*(AUV.YG*AUV.pqr(1)+AUV.uvw(3))           AUV.m*(AUV.ZG*AUV.pqr(3)+AUV.XG*AUV.pqr(1))  -AUV.m*(AUV.YG*AUV.pqr(3)-AUV.uvw(1));...
        -AUV.m*(AUV.ZG*AUV.pqr(1)-AUV.uvw(2))           -AUV.m*(AUV.ZG*AUV.pqr(2)+AUV.uvw(1))         AUV.m*(AUV.XG*AUV.pqr(1)+AUV.YG*AUV.pqr(2))];
    
    
    
CRB21 = [-AUV.m*(AUV.YG*AUV.pqr(2)+AUV.ZG*AUV.pqr(3))  AUV.m*(AUV.YG*AUV.pqr(1)+AUV.uvw(3))             AUV.m*(AUV.ZG*AUV.pqr(1)-AUV.uvw(2));...
         AUV.m*(AUV.XG*AUV.pqr(2)-AUV.uvw(3))          -AUV.m*(AUV.ZG*AUV.pqr(3)+AUV.XG*AUV.pqr(1))     AUV.m*(AUV.ZG*AUV.pqr(2)+AUV.uvw(1));...
         AUV.m*(AUV.XG*AUV.pqr(3)+AUV.uvw(2))          AUV.m*(AUV.YG*AUV.pqr(3)-AUV.uvw(1))             -AUV.m*(AUV.XG*AUV.pqr(1)+AUV.YG*AUV.pqr(2))];

CRB22 = [0     -AUV.Iyz*AUV.pqr(2)-AUV.Ixz*AUV.pqr(1)+AUV.Izz*AUV.pqr(3)  AUV.Iyz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(1)-AUV.Iyy*AUV.pqr(2);...
         AUV.Iyz*AUV.pqr(2)+AUV.Ixz*AUV.pqr(1)-AUV.Izz*AUV.pqr(3)  0    -AUV.Ixz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(2)+AUV.Ixx*AUV.pqr(1);...
         -AUV.Iyz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(1)+AUV.Iyy*AUV.pqr(2)  AUV.Ixz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(2)-AUV.Ixx*AUV.pqr(1)   0];
     
     
CRB21 = [-AUV.m*(AUV.YG*AUV.pqr(2)+AUV.ZG*AUV.pqr(3))  AUV.m*(AUV.YG*AUV.pqr(1)+AUV.uvw(3))             AUV.m*(AUV.ZG*AUV.pqr(1)-AUV.uvw(2));...
         AUV.m*(AUV.XG*AUV.pqr(2)-AUV.uvw(3))          -AUV.m*(AUV.ZG*AUV.pqr(3)+AUV.XG*AUV.pqr(1))     AUV.m*(AUV.ZG*AUV.pqr(2)+AUV.uvw(1));...
         AUV.m*(AUV.XG*AUV.pqr(3)+AUV.uvw(2))          AUV.m*(AUV.YG*AUV.pqr(3)-AUV.uvw(1))             -AUV.m*(AUV.XG*AUV.pqr(1)+AUV.YG*AUV.pqr(2))];

CRB22 = [0     -AUV.Iyz*AUV.pqr(2)-AUV.Ixz*AUV.pqr(1)+AUV.Izz*AUV.pqr(3)  AUV.Iyz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(1)-AUV.Iyy*AUV.pqr(2);...
         AUV.Iyz*AUV.pqr(2)+AUV.Ixz*AUV.pqr(1)-AUV.Izz*AUV.pqr(3)  0    -AUV.Ixz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(2)+AUV.Ixx*AUV.pqr(1);...
         -AUV.Iyz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(1)+AUV.Iyy*AUV.pqr(2)  AUV.Ixz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(2)-AUV.Ixx*AUV.pqr(1)   0];
     
          
     

CRB=[CRB11 CRB12; CRB21 CRB22];     

%%Rigi body mass
MRB11 = AUV.m*eye(3);
MRB12 = [0 AUV.m*AUV.ZG -AUV.m*AUV.YG; -AUV.m*AUV.ZG 0  AUV.m*AUV.XG; AUV.m*AUV.YG -AUV.m*AUV.XG  0];
MRB21 = [0 -AUV.m*AUV.ZG AUV.m*AUV.YG;...
         AUV.m*AUV.ZG 0 -AUV.m*AUV.XG;...
         -AUV.m*AUV.YG AUV.m*AUV.XG  0];
MRB22 = [AUV.Ixx    -AUV.Ixy   -AUV.Ixz;...
         -AUV.Ixy   AUV.Iyy   -AUV.Iyz;...
         -AUV.Ixz   -AUV.Iyz AUV.Izz];
MRB=[MRB11 MRB12; MRB21 MRB22];

%%Added mass
MA=-[AUV.X_udot AUV.Y_vdot AUV.Z_wdot AUV.K_pdot AUV.M_qdot AUV.N_rdot].*eye(6);

V=[AUV.uvw;AUV.pqr];

%%compute acceleration
accel = inv(MRB+MA)*(TAU_P+G+0*CAV*V-DV.*V-CRB*V);
accel(4,:)=0; %%restrict roll


ax_e=0;
ay_e=0;
if(flag==1)
    ax_e = 0.0000000001*randn;
    ay_e = 0.0000000001*randn;
end
AUV.dot_uvw=accel(1:3)+[ax_e; ay_e; 0];
AUV.dot_pqr=accel(4:6);
%%update speed
AUV.uvw = AUV.uvw + AUV.dot_uvw*dt;
AUV.pqr = AUV.pqr + AUV.dot_pqr*dt;
end



