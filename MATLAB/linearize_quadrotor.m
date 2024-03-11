function [A, B] = linearize_quadrotor(p, mu, I_nu,n_nu, r_nu)

    % Define symbolic variables and parameters
    syms x1 x2 x3 phi theta pssi v1 v2 v3 omega1 omega2 omega3 real
    z = [x1, x2, x3, phi, theta, pssi, v1, v2, v3, omega1, omega2, omega3];


    % Define other symbolic variables
    syms I11 I22 I33 n1 n2 n3 r1 r2 r3 u1 u2 u3 u4 real
    I = diag([I11, I22, I33]);
    n = [n1,n2,n3];
    r = [r1,r2,r3];
    u = [u1,u2,u3,u4];

    % Rotation matrix from body-fixed frame C to inertial frame E
    Rot_CE = [cos(theta)*cos(pssi), sin(phi)*sin(theta)*cos(pssi) - cos(phi)*sin(pssi), sin(phi)*sin(pssi) + cos(phi)*sin(theta)*cos(pssi);
              cos(theta)*sin(pssi), cos(phi)*cos(pssi) + sin(phi)*sin(theta)*sin(pssi), cos(phi)*sin(theta)*sin(pssi) - sin(phi)*cos(pssi);
              -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

    % Transformation matrix T_inv
    T_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi),            -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % Define the dynamics of the quadrotor system
    f = [[v1; v2; v3];
         T_inv * [omega1; omega2; omega3];
         -[0, 0, p(1)].' + (1/p(3)) * Rot_CE * ([0; 0; u(1)+u(2)+u(3)+u(4)] +  [r(1); r(2); r(3)]);
         inv(I) * ([(u(2) - u(4)) * p(2); (u(3) - u(1)) * p(2); (u(1) - u(2) + u(3) - u(4)) * p(4)] + [n(1); n(2); n(3)] - cross([omega1; omega2; omega3], I * [omega1; omega2; omega3]) )];

    % Linearize the system by calculating the Jacobian matrices
    A = jacobian(f, z);
    B = jacobian(f, u);

    % Display jacobian A, B
    disp("Jacobian of A");
    disp(A);
    disp("Jacobian of B");
    disp(B);

    for i = 1:1:4 
        A = subs(A, u(i), p(3)*p(1)/4);  % For now using mg/4
        B = subs(B, u(i), p(3)*p(1)/4);
    end
    
    for i = 4:1:12 % As x1,x2,x3 are not given 
        A = subs(A, z(i), 0);
        B = subs(B, z(i), 0);
    end
    
    for i = 1:1:3 % As n1,n2,n3 are not given 
        A = subs(A, n(i), n_nu(i));
        B = subs(B, n(i), n_nu(i));
    end
    
    for i = 1:1:3 % As r1,r2,r3 are not given 
        A = subs(A, r(i), r_nu(i));
        B = subs(B, r(i), r_nu(i));
    end

   for i = 1:1:4 % Assumed params
        A = subs(A, p(i));
        B = subs(B, p(i));
   end
    
   for i = 1:1:3 % Assumed params
       A = subs(A, I_nu(i));
       B = subs(B, I_nu(i));
   end

    % Evaluate the matrices A and B
    A = double(A);
    B = double(B);
end
