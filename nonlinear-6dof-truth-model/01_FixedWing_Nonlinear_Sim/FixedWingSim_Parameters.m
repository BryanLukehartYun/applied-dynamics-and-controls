% Parameters - Provided by a Client to stimulate their Environment
%defines aerodynamic model coefficients

%Define constants to deg-rad conversions
r2d = 180/pi;
d2r = pi/180;

%Constants for Lift, Drag, Sideforce, Yaw, Pitch and Roll Coefficients
%coefficients are converted (1/rad) expect "not" terms which are dimensionless
CLo    =  0.004608463;      %(---)
CLa    =  0.0794655/d2r;    %(1/rad)
CLq    =  0.0508476/d2r;    %(1/rad)
CLadot =  0.0;              %(1/rad)
CLde   =  0.0121988/d2r;    %(1/rad)
CLdf   =  0.0144389/d2r;    %(1/rad)

CDo    =  0.01192128;       %(---)
CDa    =  0.00550063/d2r;   %(1/rad)
CDq    =  0.00315057/d2r;   %(1/rad)
CDadot =  0.0;              %(1/rad)
CDde   = -0.000587647/d2r;  %(1/rad)
CDdf   =  0.00136385/d2r;   %(1/rad)

CYo    =  0.0;              %(---)
CYb    = -0.0219309/d2r;    %(1/rad)
CYp    =  0.00133787/d2r;   %(1/rad)
CYr    =  0.0094053/d2r;    %(1/rad)
CYda   =  0.00049355/d2r;   %(1/rad)
CYdr   =  0.00293048/d2r;   %(1/rad)

Clo    =  0.0;              %(---)
Clb    = -0.00173748/d2r;   %(1/rad)
Clp    = -0.00739342/d2r;   %(1/rad)
Clr    =  0.0000699792/d2r; %(1/rad)
Clda   = -0.00213984/d2r;   %(1/rad)
Cldr   =  0.000479021/d2r;  %(1/rad)

CMo    = -0.02092347;       %(---)
CMa    = -0.0041873/d2r;    %(1/rad)
CMq    = -0.060661/d2r;     %(1/rad)
CMadot = -0.05/d2r;         %(1/rad)
CMde   = -0.0115767/d2r;    %(1/rad)
CMdf   =  0.000580220/d2r;  %(1/rad)
CNo    =  0.0;              %(---)
CNb    =  0.00320831/d2r;   %(1/rad)
CNp    = -0.000432575/d2r;  %(1/rad)
CNr    = -0.00886783/d2r;   %(1/rad)
CNda   = -0.000206591/d2r;  %(1/rad)
CNdr   = -0.00144865/d2r;   %(1/rad)
%Constants for aircraft mass properties and configuration
Ixx    =  8890.63;          %(slugs-ft^2)
Iyy    =  71973.5;          %(slugs-ft^2)
Izz    =  77141.1;          %(slugs-ft^2)
Ixz    =  181.119;          %(slugs-ft^2)
Ixy    =  0.0;              %(slugs-ft^2)
Iyz    =  0.0;              %(slugs-ft^2)
mass   =  762.8447;         %(slugs)
g      =  32.17561865;      %(ft/sec^2)
S      =  300.0;            %(ft^2)
cbar   =  11.32;            %(ft)
b      =  30.0;             %(ft)
rho    =  0.0014962376;     %(slugs/ft^3)
%define initial state variables
Vt0    =  626.81863;        %(ft/sec)
alpha0 =  3.6102915*d2r;    %(rad)
beta0  =  0.0;              %(rad)
p0     =  0.0;              %(rad)
q0     =  0.0;              %(rad)
r0     =  0.0;              %(rad)
phi0   =  0.0;              %(rad)
theta0 =  alpha0;           %(rad)
psi0   =  0.0;              %(rad)
de0    = -3.03804303*d2r;   %(rad)  
df0    =  1.5*d2r;          %(rad)
da0    =  0.0;              %(rad)
dr0    =  0.0;              %(rad)
thrust =  3146.482666;      %(lbs)