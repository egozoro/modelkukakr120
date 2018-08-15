
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           author : Romain BAÏSSE             %
%  model robot KUKA kr120 r2700 extra HA kuka  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Length between each frame

    A = 0.240;                  %distance between 01 and 02       
    B = 0.350;                  %distance between 02 and 03    
    C = 1.150;                  %distance between 03 and 04     
    D = 0.041;                  %distance between 04 and 05 along the x4 axis  
    E = 1.2;                    %distance between 04 and 05 alons the -y4 axis
    F = 0.215;                  %distance between 05/06 and 07

% Links definition with MDH
% robot with 7 links --> PRRRRRR
    L(1) = Link([0, 0, 0, -pi/2,1], 'modified');
    L(2) = Link([0, A, 0, pi/2], 'modified');
    L(3) = Link([0, 0, B, pi/2], 'modified');
    L(4) = Link([0, 0, C, 0], 'modified');
    L(5) = Link([0, E, -D, pi/2], 'modified');
    L(6) = Link([0, 0, 0, -pi/2], 'modified');
    L(7) = Link([0, F, 0, pi/2], 'modified');

% serial robot (composed of all links in series --> SerialLink)

    kr120 = SerialLink(L, 'name', 'kr120');

% Motion range
    L(1).qlim = [0 1];
    L(2).qlim = [ -185 185];
    L(3).qlim = [ -5 -140];
    L(4).qlim = [ -120 155];
    L(5).qlim = [ -350 350];
    L(6).qlim = [ -125 125];
    L(7).qlim = [ -350 350];

% plot the robot configuration

    %kr120.plot([0 0 pi/4 pi/4 pi/4 pi/8 -pi/4])

% differents configurations to test

    %qifig = [0 pi/2 pi/2 0 0 0 0];
    q = [0 pi/2 pi/2 0 0 pi/4 0]; 
    %q = [0 0 pi/4 pi/4 pi/4 pi/8 -pi/4];
    q2 = [0.4 pi pi/2 -pi/8 -pi/2 0 0 ];
    
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% generation de trajectoire exemple %%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
%     q0=[0 0 0 0 0 0 0]; 
%     q1b=[.0873 .1745 pi/3 pi/2 .5236 pi/1.5 0];
%     q1=[.0873 .1745 .3491 .1745 .5236 .8727 0]; 
%     q2=[.0873 0 -.3491 .2618 .8727 1.2217 0];
%     q3=[-.1745 -.3491 .5236 .0873 -.1222 .3491 0]; 
%     q4=[.1745 .1745 0 -.1396 .3191 -.5236 0]; 
%     q5=[-.3491 -.3491 -.3491 0 -.0873 0 0];
%     
%        
%     qsq1=[0.46088 0.37699 0 1.31 0 1.4451 0]; 
%     qsq2=[.81681 0.56549 0 1.0681 0 1.2566 0 ]; 
%     qsq3=[2.36 0.69115 0 0.848 0 1.4451 0 ];
%     qsq4=[2.66 0.37699 0 1.31 0 1.4451 0];
%     
%     
%     t = [0:0.04:2]
%     traj1=jtraj(q0,q1,t); 
%     traj2=jtraj(q1,q2,t); 
%     traj3=jtraj(q2,q3,t); 
%     traj4=jtraj(q3,q4,t); 
%     traj5=jtraj(q4,q5,t); 
%     traj6=jtraj(q5,q0,t); 
%    
%     
%     hold on 
%     kr120.plot(traj1)
%     kr120.plot(traj2)
%     kr120.plot(traj3)
%     kr120.plot(traj4)
%     kr120.plot(traj5)
%     kr120.plot(traj6)
%     
%     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






     
% Robot offset 

    % kr120.links(1).offset = 0
    % kr120.links(2).offset = degtorad(-20)
    % kr120.links(3).offset = degtorad(120)
    % kr120.links(4).offset = degtorad(110)
    % kr120.links(5).offset = degtorad(0)
    % kr120.links(6).offset = degtorad(0)
    % kr120.links(7).offset = degtorad(0)
    %kr120.plot(qtest)
 
% computation of the forward kinematic

    %x = kr120.fkine(q)
    
% computation of the inverse kinematic 

    %q2 = kr120.ikine(X)
    %x2 = kr120.fkine(Q)  
    %kr120.plot(Q)
    
    % here we find --> x = x2 --> but q2 != q (there is some different
    % configurations for the same position (redundant robot)

% computation and plotting of the trajectory
     T1 = kr120.fkine(q)        % initial point
     T2 = kr120.fkine(q2)    % final point
     
     q1 = kr120.ikine(T1)                  % initial configuration
     q2 = kr120.ikine(T2)                  % final configuration
                                           
     t = [0:0.05:5]                        % time vector

    %vector of each intermediate
    %configuration between initial and final configuration
     q = mtraj(@tpoly, q1, q2, t)         
    
    
    kr120.plot(q)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                                   %TESTS%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% avec ces valeurs de configuration à atteindre : 
%     q = [0 pi/2 pi/2 0 0 pi/4 0]; 
%     q2 = [0.4 pi pi/2 -pi/8 -pi/2 0 0 ];
% 
% le robot peut bien aller du point A au point B mais on voit bien qu'il
% effectue une trajectoire impossible en réalité. pas de prise en compte des
% limites et de la géométrie du robot (modele CAD)
% 






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                                   %FIN TESTS%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% computation of the jacobian
    % J = jacob0(kr120 , q)
    % about(J)
     xd = [-0.148435 -0.134021 0.00234365 0 0.0 0.0]'
    % q = pinv(J) * xd
    % q' 
xDot = [-0.00742174, -0.00670104, 0.000117183, 0, 0, 0]'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% comparison between the fkine applied with the informations of the robot
% model and the matrix we found on my sheet

% homogeneous matrix between joints

    T01 = [1 0 0 0; 0 0 1 q(1); 0 -1 0 0; 0 0 0 1]
    T12 = [cos(q(2)) -sin(q(2)) 0 0; 0 0 -1 -A; sin(q(2)) cos(q(2)) 0 0; 0 0 0 1]
    T23 = [cos(q(3)) -sin(q(3)) 0 B; 0 0 -1 0; sin(q(3)) cos(q(3)) 0 0; 0 0 0 1]
    T34 = [cos(q(4)) -sin(q(4)) 0 C; sin(q(4)) cos(q(4)) 0 0; 0 0 1 0; 0 0 0 1]
    T45 = [cos(q(5)) -sin(q(5)) 0 -D; 0 0 -1 -E; sin(q(5)) cos(q(5)) 0 0; 0 0 0 1]
    T56 = [cos(q(6)) -sin(q(6)) 0 0; 0 0 1 0; -sin(q(6)) -cos(q(6)) 0 0; 0 0 0 1]
    T67 = [cos(q(7)) -sin(q(7)) 0 0; 0 0 -1 -F; sin(q(7)) cos(q(7)) 0 0; 0 0 0 1]

    display('result obtained by using the T07 matrix with the configuraiton q : ')

    
% Computation of the jacobian matrix
%  shape of the jacobian amtrix: 
% 
% 
% 
%  Jacobian = |z1(R0)     z2(R0)^O2O7(R0) z3(R0)^O3O7(R0) z4(R0)^O4O7(R0) z5(R0)^O2O7(R0) z5(R0)^O2O7(R0) z6(R0)^O6O7(R0) z7(R0)^O7O7(R0)|
%             |[O O O]'         z2(R0)      z3(R0)              z4(R0)        z5(R0)          z6(R0)          z7(R0)          z8(R0)  |


%computation  of the OiOn vectors : 

    T07 = T01*T12*T23*T34*T45*T56*T67;
    T06 = T01*T12*T23*T34*T45*T56;
    T05 = T01*T12*T23*T34*T45;
    T04 = T01*T12*T23*T34;
    T03 = T01*T12*T23;
    T02 = T01*T12;
    
    O7O7 = [0 0 0]';
    O6O7 = T06(1:3,1:3)*[0 -F 0]';                                   % R06 * 0607(R6)
    O5O7 = T05(1:3,1:3)*[0 0 0]' + O6O7;                             % O5O6(R0) + O6O7(R0) = R05 * O5O6(R5) + O6O7(R0)
    O4O7 = T04(1:3,1:3)*[-D -E 0]'+ O5O7;                            % O4O5(R0) + O5O7(R0) = R04 * O4O5(R4) + O5O7(R0) 
    O3O7 = T03(1:3,1:3)*[C 0 0]' + O4O7;                             % O3O4(R0) + O4O5(R0) = R03 * O3O4(R3) + O4O7(R0)
    O2O7 = T02(1:3,1:3)*[B 0 0]' + O3O7;                             % O2O3(R0) + O3O4(R0) = R02 * O2O3(R2) + O3O7(R0)
    

    z1 = [0 1 0]';
    z2 = [0 0 1]';
    z3 = [sin(q(2)) -cos(q(2)) 0]';                                  %here normaly it's [s2 c2 0] but we don't have the smae result if we use that miss a - 
    z4 = [sin(q(2)) -cos(q(2)) 0]';                                  %here normaly it's [s2 c2 0] but we don't have the smae result if we use that miss a - 
    z5 = [sin(q(4))*cos(q(3))*cos(q(2)) + cos(q(4))*sin(q(3))*cos(q(2)) 
            sin(q(4))*cos(q(3))*sin(q(2)) + cos(q(4))*sin(q(3))*sin(q(2))
            sin(q(4))*sin(q(3)) - cos(q(4))*cos(q(3))
            ];
    z6 = [-sin(q(5))*cos(q(4))*cos(q(3))*cos(q(2)) + sin(q(5))*sin(q(4))*sin(q(3))*cos(q(2)) + cos(q(5))*sin(q(2))
        -sin(q(5))*cos(q(4))*cos(q(3))*sin(q(2)) + sin(q(5))*sin(q(4))*sin(q(3))*sin(q(2)) + cos(q(5))*cos(q(2))
        -sin(q(5))*cos(q(4))*sin(q(3)) - sin(q(5))*sin(q(4))*cos(q(3)) 
        ];
    
    z7 = [
        sin(q(6))*cos(q(5))*cos(q(4))*cos(q(3))*cos(q(2)) - sin(q(6))*cos(q(5))*sin(q(4))*sin(q(3))*cos(q(2)) + cos(q(6))*sin(q(4))*cos(q(3))*cos(q(2)) + cos(q(6))*cos(q(4))*sin(q(3))*cos(q(2)) + sin(q(6))*sin(q(5))*sin(q(2))
        -(-sin(q(6))*cos(q(5))*sin(q(4))*sin(q(3))*sin(q(2)) + sin(q(6))*cos(q(5))*cos(q(4))*cos(q(3))*sin(q(2)) + cos(q(6))*sin(q(4))*cos(q(3))*sin(q(2)) + cos(q(6))*cos(q(4))*sin(q(3))*sin(q(2)) + sin(q(6))*sin(q(5))*cos(q(2)))
        sin(q(6))*cos(q(5))*cos(q(4))*sin(q(3)) + sin(q(6))*cos(q(5))*sin(q(4))*cos(q(3)) + cos(q(6))*sin(q(4))*sin(q(3)) - cos(q(6))*cos(q(4))*cos(q(3))
        ];

%attention : z7 2nd row there is not supposed to be the ' - ' in front of all the equation    
    
%expression of the jacobian matrix
    JACOB = [z1 cross(z2,O2O7) cross(z3,O3O7) cross(z4,O4O7) cross(z5,O5O7) cross(z6,O6O7) cross(z7,O7O7);
        [0 0 0]' z2 z3 z4 z5 z6 z7
        ]
    
% RESULTS : T07 on the configuration q give the same results as the fkine
% function (tests realised on some configurations)
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% computation of the "jacobienne preferentielle" on my sheet
% 
%     Jacobienne = [sin(q(2))*cos(q(3))*cos(q(4))-sin(q(2))*sin(q(3))*sin(q(4)) 0 E E 0 0 -F*sin(q(6))*sin(q(5));
%                  -sin(q(2))*cos(q(3))*sin(q(4))-sin(q(2))*sin(q(3))*cos(q(4)) 0 C -D 0 0 0;
%                  cos(q(2)) -(B+E)*(sin(q(3))*cos(q(4))+cos(q(3))*sin(q(4)))-(C*(-sin(q(3))*sin(q(4))+cos(q(3))*cos(q(4)))) 0 0 0 0 F*sin(q(6))*cos(q(5));
%                  0 sin(q(3))*cos(q(4))+cos(q(3))*sin(q(4)) 0 0 0 -sin(q(5)) sin(q(6))*cos(q(5));
%                  0 -sin(q(3))*sin(q(4))+cos(q(3))*cos(q(4)) 0 0 -1 0 -cos(q(6));
%                  0 0 1 1 0 -cos(q(5)) sin(q(6))*sin(q(5));
%                  ]
           
% methods of the Robotic Toolbox  -->  which one is the best ?
    %Jacob = kr120.jacobe(q)
    Jacob2 = kr120.jacob0(q)
    %Jacob3 = kr120.jacobn(q)   
    
% computation of the pseudo jacobian : 

    J_transpose = transpose(JACOB)
    JACOB_J_transpose_inv = inv(JACOB*J_transpose);
    
    J_pseudo = J_transpose*JACOB_J_transpose_inv
    
    
% the jacobian obtained is not square so we have to use the pseudo-inverse
% jacobian
   %Jacobienneinv =pinv(Jacobienne)
    Jacobienne_inv = pinv(JACOB)
    Jacobienne_inv2 = pinv(Jacob2)
    jsingu(JACOB)
    jsingu(Jacob2)
    
% RESULTS : 
    % the function of the matlab toolbox pinv uses the pseudo_jacobian
    % method. We found the same result when decomposing the method or just
    % calling the function
%     
  xDot
  qd = Jacobienne_inv * xDot
  qd2 = Jacobienne_inv2 * xDot
  
%  qd2 = kr120.ikine(T)

