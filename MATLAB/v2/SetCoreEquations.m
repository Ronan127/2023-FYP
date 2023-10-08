load symbols.mat
%%
eqns_force= unwrap_y([ ...
    m1*a1 == f2+f3+f4+f5+f6+f7+w1; ...
    m2*a2 == w2-f2+gf3+gf4; ...
    m3*a3 == w3-f3-gf3+gf5; ...
    m4*a4 == w4-f4-gf4+gf6; ...
    m5*a5 == w5-f5-gf5+F_react5; ...
    m6*a6 == w6-f6-gf6+F_react6; ...
    m7*a7==-f7+F_react7+w7 ...
    ]);
eqns_weight = unwrap_y(w==m.*g);
assignmnent_g = unwrap_y(g2 == 9.81);
assignment_gearForce = unwrap_y(gf == 0);
assignment_reaction = unwrap_y([F_react53 == 0, F_react63 == 0, F_react73 == 0]);
assignment_accelleration = unwrap_y(a == 0);

%%

%%
Rvector2 = [sin(t13); cos(t13); 0];
eqns_gearForce = unwrap_y([gf1==0, gf2==0, gf3==gfa3*Rvector2, gf4==gfa4*Rvector2, gf5==gfa5*Rvector2, gf6==gfa6*Rvector2, gf7 == 0]);
assignment_gearForceAbsolute = unwrap_y(gfa == 0);

%%
moment1=I1*-ta1 == cross(l3,f3) + cross(l4,f4) + cross(l5,f5) + cross(l6,f6);
moment7=I7*-ta7 == T + cross(l7*ratio7Weight, w7) + cross(l7*ratio7Contact, F_react7);

assignment_forces= unwrap_y(f13 == 0);
assignment_angularAccelleration = unwrap_y(ta == 0);
assignment_torque = T_z == 0;
assignment_momentReaction = [M_react63 == 0];
eqns_tailGroundFriction = [F_react71 ==  -C_friction7*F_react72];



%%
eqns_moments = unwrap_y([
    I1*-ta1 == cross(l3,f3) + cross(l4,f4) + cross(l5,f5) + cross(l6,f6);
    I2*ta23 == T_z - r2*gfa3 + r2*gfa4;
    I3*ta33 == -r3 * gfa3 - r3* gfa5;
    I4*ta43 == r4 * gfa4 + r4* gfa6;
    I5*ta53 == -r5 * gfa5 + M_react53;
    I6*ta63 == r6 * gfa6 + M_react63;
    I7*-ta7 == T + cross(l7/2, w7) + cross(l7, F_react7)
    ]);

eqns_accelleration = [a1 == a2;
    a2 == a5 + cross(-l5, ta1) + cross(cross(tv1,-l5),tv1);
    a3 == a2 + cross(l3, ta1) + cross(cross(tv1,l3),tv1);
    a4 == a2 + cross(l4, ta1) + cross(cross(tv1,l4),tv1);
    a5 == a2 + cross(l5, ta1) + cross(cross(tv1,l5),tv1);
    a6 == a2 + cross(l6, ta1) + cross(cross(tv1,l6),tv1);
    a7==  a1 + cross(l7/2, ta7)];

assignment_angularVelocity = unwrap_y(tv == 0);

assignment_radius = [r2==0.070*50/(50+33)
    r3==0.041*33/(16+33)
    r4==r3
    r5==0.041*16/(16+33)
    r6==r5];

assignment_lengths = [l(1,1) == 0;
    l(1,2)==0; 
    l(1,3)==0.07;
    l(1,4)==-0.07;
    l(1,5)==0.041;
    l(1,6)==-0.041;
    l(1,7)==-0.425;
    unwrap_y(l(2:3,:) == 0)];


%%
condition_2d = [unwrap_y(a(3,:) == 0);
    unwrap_y(v(3,:) == 0);
    unwrap_y(s(3,:) == 0);
    unwrap_y(ta(1:2,:) == 0);
    unwrap_y(tv(1:2,:) == 0);
    unwrap_y(t(1:2,:) == 0);
    ];

%%
eqns_angularAcelleration = [ta3-ta1 == -r2/r3*(ta2-ta1);
    ta4-ta1 == -r2/r3*(ta2-ta1);
    ta5-ta1 == -r3/r5*(ta3-ta1);
    ta6-ta1 == -r3/r5*(ta4-ta1);
    ];

Rvector = [cos(t13); -sin(t13); 0];
Rvector2 = [sin(t13); cos(t13); 0];
Rvector7 = [cos(t73); -sin(t73); 0];
eqns_lengths = [l1==0;
    l2==0;
    l3==l3_ab*Rvector;
    l4==-l3_ab*Rvector;
    l5==l5_ab*Rvector;
    l6==-l5_ab*Rvector;
    l7==l7_ab*Rvector7;
    t73 == pi-ot73
    separation23 == l3_ab;
    separation35==l5_ab-l3_ab;
    ];

%eqns_lengths(20) = isolate(eqns_lengths(20), ot73);

eqns_radius = [r2==separation23*N_sun/(N_sun+N_idler)
    r3==separation35*N_idler/(N_planet+N_idler)
    r4==r3
    r5==separation35*N_planet/(N_planet+N_idler)
    r6==r5];


%%
assignment_angle = [unwrap_y(t(:,1:6) == 0); t(1:2,7)==0];

eqns_displacement = [s1 == s2;
    s2 == s5 + -l5;
    s3 == s2 + l3;
    s4 == s2 + l4;
    s5 == s2 + l5;
    s6 == s2 + l6;
    s7 == s2 + l7/2;
    s7End == s2 + l7;
    ];

%ta73 = - d^2/dt^2 (arcsin(l72(t)/l7_ab))
%From wolfram alpha, giving y = -ta73, x(t) = l72(t), c = l7_ab, x1 = l72' = -v22, x2 = l72'' = -a22
syms y x x1 x2 c;
tailKinematics = y == ((c^2-x^2)*x2+x*x1^2)/(c^3*(1-x^2/c^2)^(3/2));
tailKinematics = subs(tailKinematics, y, -ta73);
tailKinematics = subs(tailKinematics, x, l72);
tailKinematics = subs(tailKinematics, x1, -v22);
tailKinematics = subs(tailKinematics, x2, -a22);



condition_tailOnGround = [s7End2 == 0; 
    ratio7Contact == 1; 
    eqns_tailGroundFriction;
    tailKinematics;
    ];





%%
assignmnent_masses = [m7 == 0.43095538069190569841;
    m2 == 2*0.056167152496905625725;
    m1 == 2*0.13271340229034475633;
    m3 == 2*0.018874380082186052732;
    m4 == m3;
    m5 == 2*0.05463272607293223615;
    m6 == m5;
    ratio7Weight == 0.25;
    ];
assignment_inertias = [I2 == 2*3.437592086857546033e-05;
    I5 == 2*6.116925860141665791e-05;
    I6 == I5;
    I1 == 2*0.00076846598232952812758;
    I3 == 2*6.6879087944629165118e-06;
    I4 == I3;
    I7 == 3279.681e-6; %From onshape
    ];


%%
%rolling on step, back wheel lifting, tail on step

Rmatrix7 = [
    -cos(t73), -sin(t73), 0;
    sin(t73), -cos(t73), 0;
    0, 0, 1
    ];

%ta73 = - d^2/dt^2(tan^(-1)(u2/u1))
%From wolfram alpha, giving z = -ta73, y(t) = u2(t),  y1 = u2' = -v22, y2 = u2'' = -a22, x(t) = u1(t), x1 = u1' = -v21, x2 = u1'' = -a21
syms z y y1 y2 x x1 x2;
tailKinematics = z == 1/(x^2+y^2)^2*(x*y*(2*x1^2+y*y2-2*y1^2)-x^2*(y*x2+2*x1*y1)+y^2*(2*x1*y1-y*x2)+x^3*y2);
tailKinematics = subs(tailKinematics, z, -ta73);
tailKinematics = subs(tailKinematics, y, u2);
tailKinematics = subs(tailKinematics, y1, -v22);
tailKinematics = subs(tailKinematics, y2, -a22);
tailKinematics = subs(tailKinematics, x, u1);
tailKinematics = subs(tailKinematics, x1, -v21);
tailKinematics = subs(tailKinematics, x2, -a21);

condition_tailOnStep = [
    tan(-t73) == u2/u1;
    ratio7Contact == u2/l72;
    tailKinematics;
    u1 == step_width*(N2-1) - s11;
    u2 == step_height*N2 - s12;
    F_reactPrime71 == - F_reactPrime72 * C_friction7
    F_react7 == Rmatrix7 * F_reactPrime7 
    ];
assignment_v = unwrap_y([ ...
    v(1,:) == 1; ...
    v(2,:) == 0]);
eqns_velocity = [
    v1 == v2;
    v2 == v5 + cross(-l5, tv1);
    v3 == v2 + cross(l3, tv1);
    v4 == v2 + cross(l4, tv1);
    v5 == v2 + cross(l5, tv1);
    v6 == v2 + cross(l6, tv1);
    ];
eqns_angularVelocity = [
    tv3-tv1 == -r2/r3*(tv2-tv1);
    tv4-tv1 == -r2/r3*(tv2-tv1);
    tv5-tv1 == -r3/r5*(tv3-tv1);
    tv6-tv1 == -r3/r5*(tv4-tv1);
    ];
condition_rolling = [
    a51 == ta53*rw; 
    a52 == 0; 
    M_react53 == - F_react51 * rw;
    v51 == tv53*rw;
    v52 == 0;
    s51 == t53*rw;
    C_friction5 == F_react51/F_react52;
    ];

eqns_angle = [t3-t1 == -r2/r3*(t2-t1);
    t4-t1 == -r2/r3*(t2-t1);
    t5-t1 == -r3/r5*(t3-t1);
    t6-t1 == -r3/r5*(t4-t1);
    ];
condition_wheel2NoContact = [F_react73 == 0; F_react6 == 0; M_react6==0;];
assignment_gears = [separation35 == 0.041; separation23 == 0.07; rw == 0.075; l7_ab == 0.425;
    N_idler == 33; N_sun == 50; N_planet == 16; C_friction7 == 0.0;];
assignment_steps = [step_start == [rw; 0; 0]; step_height == 0.15; step_width == 0.2;];

condition_wheel1locked = [a5 == 0; ta5 == 0; v5 == 0; tv5 == 0];
condition_angle_quadrants = [
    -pi/2 <= t13; t13 <= pi/2;
    pi/2 <= t73; t73 <= pi;];
%%
Core_equations=[eqns_weight; eqns_force;  eqns_gearForce; eqns_moments;  eqns_lengths; eqns_radius;
    eqns_accelleration; eqns_velocity; eqns_displacement; 
    eqns_angularAcelleration; eqns_angularVelocity; eqns_angle;
    ]; 


save CoreEquations.mat Core_equations condition_2d condition_tailOnStep condition_rolling condition_wheel2NoContact condition_wheel1locked condition_tailOnGround condition_angle_quadrants eqns_radius