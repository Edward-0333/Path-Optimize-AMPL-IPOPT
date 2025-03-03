param Nfe;
param K_radau;

set I := {1..Nfe};
set I1:= {1..Nfe-1};
set J := {1..K_radau};
set K := {0..K_radau};

param tau{j in K};
param dljtauk{j in K,k in K};

param BV{i in {1..6}};
param Nobs;
param amax;
param vmax;
param wmax;
param phymax;
param L_wheelbase;

param tf;
param hi = tf / Nfe;

param OV{i in {1..Nobs}, j in I, k in K, m in {1..7}, n in {1..2}};
param OC{i in {1..Nobs}, j in I, k in K, n in {1..2}};
param Nov{i in {1..Nobs}};
param Area{i in {1..Nobs}};

param AreaVehicle;

var x{i in I, j in K};
var y{i in I, j in K};
var theta{i in I, j in K};
var v{i in I, j in K};
var phy{i in I, j in K};
var a{i in I, j in J};
var w{i in I, j in J};
var egoV{i in I, j in K, m in {1..4}, n in {1..2}};

minimize objective_:
tf + sum{i in I, j in J}(a[i,j]^2 + w[i,j]^2) + sum{i in I, k in {1..Nobs}}(exp(-2.0*((x[i,0] - OC[k,i,0,1])^2 + (y[i,0] - OC[k,i,0,2])^2)));

s.t. DIFF_dxdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*x[i,j]) = hi * v[i,k] * cos(theta[i,k]);
s.t. DIFF_dydt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*y[i,j]) = hi * v[i,k] * sin(theta[i,k]);
s.t. DIFF_dtdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*theta[i,j]) = hi * tan(phy[i,k]) * v[i,k] / L_wheelbase;
s.t. DIFF_dvdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*v[i,j]) = hi * a[i,k];
s.t. DIFF_dpdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*phy[i,j]) = hi * w[i,k];
s.t. EQ_diffx {i in I1}:
x[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*x[i,j]);
s.t. EQ_diffy {i in I1}:
y[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*y[i,j]);
s.t. EQ_difftheta {i in I1}:
theta[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*theta[i,j]);
s.t. EQ_diffv {i in I1}:
v[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*v[i,j]);
s.t. EQ_diffphy {i in I1}:
phy[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*phy[i,j]);

s.t. RELATIONSHIP_AX {i in I, j in K}:
egoV[i,j,1,1] = x[i,j] + 3.76 * cos(theta[i,j]) - 0.971 * sin(theta[i,j]);
s.t. RELATIONSHIP_BX {i in I, j in K}:
egoV[i,j,2,1] = x[i,j] + 3.76 * cos(theta[i,j]) + 0.971 * sin(theta[i,j]);
s.t. RELATIONSHIP_CX {i in I, j in K}:
egoV[i,j,3,1] = x[i,j] - 0.929 * cos(theta[i,j]) + 0.971 * sin(theta[i,j]);
s.t. RELATIONSHIP_DX {i in I, j in K}:
egoV[i,j,4,1] = x[i,j] - 0.929 * cos(theta[i,j]) - 0.971 * sin(theta[i,j]);
s.t. RELATIONSHIP_AY {i in I, j in K}:
egoV[i,j,1,2] = y[i,j] + 3.76 * sin(theta[i,j]) + 0.971 * cos(theta[i,j]);
s.t. RELATIONSHIP_BY {i in I, j in K}:
egoV[i,j,2,2] = y[i,j] + 3.76 * sin(theta[i,j]) - 0.971 * cos(theta[i,j]);
s.t. RELATIONSHIP_CY {i in I, j in K}:
egoV[i,j,3,2] = y[i,j] - 0.929 * sin(theta[i,j]) - 0.971 * cos(theta[i,j]);
s.t. RELATIONSHIP_DY {i in I, j in K}:
egoV[i,j,4,2] = y[i,j] - 0.929 * sin(theta[i,j]) + 0.971 * cos(theta[i,j]);


s.t. ObsVertexOutOfABCD {i in I, m in {1..3}, j in {1..Nobs}, k in {1..Nov[j]}}:
0.5 * abs(OV[j,i,m,k,1] * egoV[i,m,1,2] + egoV[i,m,1,1] * egoV[i,m,2,2] + egoV[i,m,2,1] * OV[j,i,m,k,2] - OV[j,i,m,k,1] * egoV[i,m,2,2] - egoV[i,m,1,1] * OV[j,i,m,k,2] - egoV[i,m,2,1] * egoV[i,m,1,2]) + 
0.5 * abs(OV[j,i,m,k,1] * egoV[i,m,3,2] + egoV[i,m,3,1] * egoV[i,m,2,2] + egoV[i,m,2,1] * OV[j,i,m,k,2] - OV[j,i,m,k,1] * egoV[i,m,2,2] - egoV[i,m,3,1] * OV[j,i,m,k,2] - egoV[i,m,2,1] * egoV[i,m,3,2]) + 
0.5 * abs(OV[j,i,m,k,1] * egoV[i,m,3,2] + egoV[i,m,3,1] * egoV[i,m,4,2] + egoV[i,m,4,1] * OV[j,i,m,k,2] - OV[j,i,m,k,1] * egoV[i,m,4,2] - egoV[i,m,3,1] * OV[j,i,m,k,2] - egoV[i,m,4,1] * egoV[i,m,3,2]) + 
0.5 * abs(OV[j,i,m,k,1] * egoV[i,m,1,2] + egoV[i,m,1,1] * egoV[i,m,4,2] + egoV[i,m,4,1] * OV[j,i,m,k,2] - OV[j,i,m,k,1] * egoV[i,m,4,2] - egoV[i,m,1,1] * OV[j,i,m,k,2] - egoV[i,m,4,1] * egoV[i,m,1,2]) >= AreaVehicle + 0.1;

s.t. CarVertexOutOfObstacle {i in I, f in {1..3}, j in {1..Nobs}, m in {1..4}}:
sum{k in {1..Nov[j]-1}}(0.5 * abs(egoV[i,f,m,1] * OV[j,i,f,k,2] + OV[j,i,f,k,1] * OV[j,i,f,k+1,2] + OV[j,i,f,k+1,1] * egoV[i,f,m,2] - egoV[i,f,m,1] * OV[j,i,f,k+1,2] - OV[j,i,f,k,1] * egoV[i,f,m,2] - OV[j,i,f,k+1,1] * OV[j,i,f,k,2])) + 0.5 * abs(egoV[i,f,m,1] * OV[j,i,f,Nov[j],2] + OV[j,i,f,Nov[j],1] * OV[j,i,f,1,2] + OV[j,i,f,1,1] * egoV[i,f,m,2] - egoV[i,f,m,1] * OV[j,i,f,1,2] - OV[j,i,f,Nov[j],1] * egoV[i,f,m,2] - OV[j,i,f,1,1] * OV[j,i,f,Nov[j],2]) >= Area[j] + 0.1;


s.t. EQ_init_x:
x[1,0] = BV[1];
s.t. EQ_init_y:
y[1,0] = BV[2];
s.t. EQ_init_theta:
theta[1,0] = BV[3];
s.t. EQ_init_v:
v[1,0] = 0;
s.t. EQ_init_phy:
phy[1,0] = 0;
s.t. EQ_terminal_x:
x[Nfe,K_radau] = BV[4];
s.t. EQ_terminal_y:
y[Nfe,K_radau] = BV[5];
s.t. EQ_terminal_theta_sin:
sin(theta[Nfe,K_radau]) = sin(BV[6]);
s.t. EQ_terminal_theta_cos:
cos(theta[Nfe,K_radau]) = cos(BV[6]);
s.t. EQ_terminal_v:
v[Nfe,K_radau] = 0;
s.t. EQ_terminal_phy:
phy[Nfe,K_radau] = 0;
s.t. EQ_terminal_a:
a[Nfe,K_radau] = 0;
s.t. EQ_terminal_w:
w[Nfe,K_radau] = 0;

s.t. Bonds_phy {i in I, j in J}:
-phymax <= phy[i,j] <= phymax;
s.t. Bonds_a {i in I, j in J}:
-amax <= a[i,j] <= amax;
s.t. Bonds_v {i in I, j in J}:
-vmax <= v[i,j] <= vmax;
s.t. Bonds_w {i in I, j in J}:
-wmax <= w[i,j] <= wmax;
s.t. Bonds_x {i in I, j in J}:
-20 <= x[i,j] <= 100;
s.t. Bonds_y {i in I, j in J}:
-20 <= y[i,j] <= 100;
