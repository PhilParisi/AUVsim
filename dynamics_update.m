function AUV = dynamics_update(AUV,dt,TAU_P,flag)

%%%%AUV dyanmics%%%%%

%%resotring force matrix
G = [ (AUV.Bf - AUV.Wt)* sin(AUV.rpy(2)); ...
       (AUV.Wt-AUV.Bf) * cos(AUV.rpy(2)) * sin(AUV.rpy(1));...
       (AUV.Wt - AUV.Bf) * cos(AUV.rpy(2))* cos(AUV.rpy(1));...
       (AUV.yg*AUV.Wt-AUV.yb*AUV.Bf) * cos(AUV.rpy(2)) * cos(AUV.rpy(1)) - (-AUV.zb*AUV.Bf+AUV.zg*AUV.Wt)*cos(AUV.rpy(2))*sin(AUV.rpy(1));...
      -(AUV.zg*AUV.Wt - AUV.zb*AUV.Bf) * sin(AUV.rpy(2)) + (AUV.xb*AUV.Bf-AUV.xg*AUV.Wt) * cos(AUV.rpy(2)) * cos(AUV.rpy(1));...
       (AUV.xg*AUV.Wt-AUV.xb*AUV.Bf) * cos(AUV.rpy(2)) * sin(AUV.rpy(1)) + (AUV.yg*AUV.Wt-AUV.yb*AUV.Bf) *sin(AUV.rpy(2))];
          
%%Added inertia
CAV11= 0*eye(3);  %%zero matrix
CAV12 = [0                      AUV.Zwdot*AUV.uvw(3)     -AUV.Yvdot*AUV.uvw(2);...
         -AUV.Zwdot*AUV.uvw(3)                      0      AUV.Xudot*AUV.uvw(1);...
        AUV.Yvdot*AUV.uvw(2)   -AUV.Xudot*AUV.uvw(1)                        0];
CAV21 = CAV12;
     
CAV22 = [0                       AUV.Nrdot*AUV.pqr(3)     -AUV.Mqdot*AUV.pqr(2);...
          -AUV.Nrdot*AUV.pqr(3)                      0       AUV.Kpdot*AUV.pqr(1);...
         AUV.Mqdot*AUV.pqr(2)  -AUV.Kpdot*AUV.pqr(1)                          0];


CAV = [CAV11 CAV12; CAV21 CAV22];

%%%Damping
DV=[AUV.Xusq*abs(AUV.uvw(1));...
   AUV.Yvsq*abs(AUV.uvw(2));...
   AUV.Zwsq*abs(AUV.uvw(3));...
   AUV.Kpsq*abs(AUV.pqr(1));...
   AUV.Mqsq*abs(AUV.pqr(2));...
   AUV.Nrsq*abs(AUV.pqr(3))];

%%rigid body inertia
CRB11 = 0*eye(3);

CRB12 = [AUV.m*(AUV.yg*AUV.pqr(2)+AUV.zg*AUV.pqr(3))    -AUV.m*(AUV.xg*AUV.pqr(2)-AUV.uvw(3))         -AUV.m*(AUV.xg*AUV.pqr(3)+AUV.uvw(2));...
        -AUV.m*(AUV.yg*AUV.pqr(1)+AUV.uvw(3))           AUV.m*(AUV.zg*AUV.pqr(3)+AUV.xg*AUV.pqr(1))  -AUV.m*(AUV.yg*AUV.pqr(3)-AUV.uvw(1));...
        -AUV.m*(AUV.zg*AUV.pqr(1)-AUV.uvw(2))           -AUV.m*(AUV.zg*AUV.pqr(2)+AUV.uvw(1))         AUV.m*(AUV.xg*AUV.pqr(1)+AUV.yg*AUV.pqr(2))];
    
    
    
CRB21 = [-AUV.m*(AUV.yg*AUV.pqr(2)+AUV.zg*AUV.pqr(3))  AUV.m*(AUV.yg*AUV.pqr(1)+AUV.uvw(3))             AUV.m*(AUV.zg*AUV.pqr(1)-AUV.uvw(2));...
         AUV.m*(AUV.xg*AUV.pqr(2)-AUV.uvw(3))          -AUV.m*(AUV.zg*AUV.pqr(3)+AUV.xg*AUV.pqr(1))     AUV.m*(AUV.zg*AUV.pqr(2)+AUV.uvw(1));...
         AUV.m*(AUV.xg*AUV.pqr(3)+AUV.uvw(2))          AUV.m*(AUV.yg*AUV.pqr(3)-AUV.uvw(1))             -AUV.m*(AUV.xg*AUV.pqr(1)+AUV.yg*AUV.pqr(2))];

CRB22 = [0     -AUV.Iyz*AUV.pqr(2)-AUV.Ixz*AUV.pqr(1)+AUV.Izz*AUV.pqr(3)  AUV.Iyz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(1)-AUV.Iyy*AUV.pqr(2);...
         AUV.Iyz*AUV.pqr(2)+AUV.Ixz*AUV.pqr(1)-AUV.Izz*AUV.pqr(3)  0    -AUV.Ixz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(2)+AUV.Ixx*AUV.pqr(1);...
         -AUV.Iyz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(1)+AUV.Iyy*AUV.pqr(2)  AUV.Ixz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(2)-AUV.Ixx*AUV.pqr(1)   0];
     
     
CRB21 = [-AUV.m*(AUV.yg*AUV.pqr(2)+AUV.zg*AUV.pqr(3))  AUV.m*(AUV.yg*AUV.pqr(1)+AUV.uvw(3))             AUV.m*(AUV.zg*AUV.pqr(1)-AUV.uvw(2));...
         AUV.m*(AUV.xg*AUV.pqr(2)-AUV.uvw(3))          -AUV.m*(AUV.zg*AUV.pqr(3)+AUV.xg*AUV.pqr(1))     AUV.m*(AUV.zg*AUV.pqr(2)+AUV.uvw(1));...
         AUV.m*(AUV.xg*AUV.pqr(3)+AUV.uvw(2))          AUV.m*(AUV.yg*AUV.pqr(3)-AUV.uvw(1))             -AUV.m*(AUV.xg*AUV.pqr(1)+AUV.yg*AUV.pqr(2))];

CRB22 = [0     -AUV.Iyz*AUV.pqr(2)-AUV.Ixz*AUV.pqr(1)+AUV.Izz*AUV.pqr(3)  AUV.Iyz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(1)-AUV.Iyy*AUV.pqr(2);...
         AUV.Iyz*AUV.pqr(2)+AUV.Ixz*AUV.pqr(1)-AUV.Izz*AUV.pqr(3)  0    -AUV.Ixz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(2)+AUV.Ixx*AUV.pqr(1);...
         -AUV.Iyz*AUV.pqr(3)-AUV.Ixy*AUV.pqr(1)+AUV.Iyy*AUV.pqr(2)  AUV.Ixz*AUV.pqr(3)+AUV.Ixy*AUV.pqr(2)-AUV.Ixx*AUV.pqr(1)   0];
     
          
     

CRB=[CRB11 CRB12; CRB21 CRB22];     

%%Rigi body mass
MRB11 = AUV.m*eye(3);
MRB12 = [0 AUV.m*AUV.zg -AUV.m*AUV.yg; -AUV.m*AUV.zg 0  AUV.m*AUV.xg; AUV.m*AUV.yg -AUV.m*AUV.xg  0];
MRB21 = [0 -AUV.m*AUV.zg AUV.m*AUV.yg;...
         AUV.m*AUV.zg 0 -AUV.m*AUV.xg;...
         -AUV.m*AUV.yg AUV.m*AUV.xg  0];
MRB22 = [AUV.Ixx    -AUV.Ixy   -AUV.Ixz;...
         -AUV.Ixy   AUV.Iyy   -AUV.Iyz;...
         -AUV.Ixz   -AUV.Iyz AUV.Izz];
MRB=[MRB11 MRB12; MRB21 MRB22];

%%Added mass
MA=-[AUV.Xudot AUV.Yvdot AUV.Zwdot AUV.Kpdot AUV.Mqdot AUV.Nrdot].*eye(6);

V=[AUV.uvw;AUV.pqr];

%%compute acceleration
accel = inv(MRB+MA)*(G+0*CAV*V-DV.*V-CRB*V); %%TAU_P+
accel(4,:)=0; %%restrict roll


ax_e=0;
ay_e=0;
%%if(flag==1)
%%ax_e = 0.0000000001*randn;
%%ay_e = 0.0000000001*randn;
%%end
AUV.dot_uvw=accel(1:3)+[ax_e; ay_e; 0];
AUV.dot_pqr=accel(4:6);
%%update speed
AUV.uvw = AUV.uvw + AUV.dot_uvw*dt;
AUV.pqr = AUV.pqr + AUV.dot_pqr*dt;
end



