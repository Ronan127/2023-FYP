clear
syms m [1 7]; %masses of elements
syms I [1 7]; %moments of inertia
syms f1 f2 f3 f4 f5 f6 f7 [3,1]; %forces applied to frame from element
syms s1 s2 s3 s4 s5 s6 s7 s7End [3,1]; %position of each element
syms v1 v2 v3 v4 v5 v6 v7 [3,1]; %position of each element
syms a1 a2 a3 a4 a5 a6 a7 [3,1]; %position of each element
syms t1 t2 t3 t4 t5 t6 t7 ot7 [3,1]; %angle of each element
syms tv1 tv2 tv3 tv4 tv5 tv6 tv7 [3,1];
syms ta1 ta2 ta3 ta4 ta5 ta6 ta7 [3,1];
syms w1 w2 w3 w4 w5 w6 w7 [3,1]; %weight of each element
syms gf1 gf2 gf3 gf4 gf5 gf6 gf7 [3,1]; %force on outer gear acting on inner gear
syms l1 l2 l3 l4 l5 l6 l7 lcontact [3,1]; %distance from centre to element
syms r [1 7]; %radius of each element
syms gfa [1 7]; %absolute gear force on each eement

syms F_react5g F_react5w [3,1]
syms T_z l3_ab l5_ab Rz Rvector g2 rw l7_ab C_friction7 C_friction5 separation23 separation35 N_sun N_idler N_planet cost13 sint13 cost73 sint73 L_body_centre;
syms step_start [3,1];
syms step_height step_width ratio7Weight ratio7Contact;
syms T [3,1];
syms F_react5 [3,1];
syms M_react5 [3,1];
syms F_react6 [3,1];
syms M_react6 [3,1];
syms F_react7 F_reactPrime7 [3,1];
syms u1 u2 N N2;

f=sym([f1, f2, f3, f4, f5, f6, f7]);
gf=sym([gf1, gf2, gf3, gf4, gf5, gf6, gf7]);
s=sym([s1, s2, s3, s4, s5, s6, s7]);
v=sym([v1, v2, v3, v4, v5, v6, v7]);
a=sym([a1, a2, a3, a4, a5, a6, a7]);
t=sym([t1, t2, t3, t4, t5, t6, t7]);
tv=sym([tv1, tv2, tv3, tv4, tv5, tv6, tv7]);
ta=sym([ta1, ta2, ta3, ta4, ta5, ta6, ta7]);
l=sym([l1, l2, l3, l4, l5, l6, l7]);
w=sym([w1, w2, w3, w4, w5, w6, w7]);

g= [0, -g2, 0].';
g=sym(g);

P = sym(pi);
%t = [zeros(1,7); zeros(1,7); t_z]
%tv = [zeros(1,7); zeros(1,7); tv_z]
%ta = [zeros(1,7); zeros(1,7); ta_z]

T = [0; 0; T_z];

save symbols.mat *