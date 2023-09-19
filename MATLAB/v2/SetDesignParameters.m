load symbols.mat

assignmnent_g = unwrap_y(g2 == 9.81);

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


assignment_gears = [separation35 == 0.041; separation23 == 0.07; rw == 0.075; l7_ab == 0.425;
    N_idler == 33; N_sun == 50; N_planet == 16; C_friction7 == 0.0;];
assignment_steps = [step_start == [0; 0; 0]; step_height == 0.15; step_width == 0.2;];

DesignParameters = [assignmnent_g; assignmnent_masses; assignment_inertias; assignment_gears; assignment_steps];


save DesignParameters.mat DesignParameters assignmnent_g assignmnent_masses assignment_inertias assignment_gears assignment_steps