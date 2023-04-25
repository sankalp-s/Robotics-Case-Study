function output = output(n, h, ax, l0, lf, d1, d2, m, p1, p2) % calcuates all  the values thetaa 
   
    % defining x, y, and z a
    [x, y, z] = deal(1, 2, 3);
    a(x) = ax;

    
    %% CONSTANT CALCULATIONS
    TOTAL_Deg = 180/pi; % radians to degrees  
    
    %in a triangle, calculation normal vectors nab, nac, and nbc for each side 
    NOR_ab = [sqrt(3)/2, -1/2, 0];
    NOR_ac = [sqrt(3)/2, 1/2, 0];
    NOR_bc = [0, 1, 0];
    
    % intermediate variables
    T1 = (lf^2*sqrt(3))/2;
    U1 = sqrt(l0^2+d1^2)*sin((2*pi/3)-atan(l0/d1)); 
    
    % calculation of  points a10, a20, b10, b20, c10, and c20
    A10 = [d2-U1*sqrt(3), -U1-d2*sqrt(3), 0]*0.5; 
    A20 = [-A10(x), A10(y), 0];
    B10 = [U1*sqrt(3)+d2, d2*sqrt(3)-U1, 0]*.5;
    B20 = [d2, U1, 0];
    C10 = [-B20(x), B20(y), 0];
    C20 = [-B10(x), B10(y), 0];
    
    % calculation of vectors, ab, ac, and bc
    AB = A20 - B10;
    AC = A10 - C20;
    BC = B20 - C10;
    

    %% STAGE 1 CALCULATIONS
   
    
    % af components
    E(1) = a(x)-h(x); 
    a(z) = ((n(y)*sqrt(lf^2*(1-n(x)^2)-E(1)^2)-n(z)*n(x)*E(1))/(1-n(x)^2))+h(z); 
    G(1) = a(z)-h(z); 
    a(y) = h(y)-sqrt(lf^2-G(1)^2-E(1)^2); 
    k(1) = a(y)-h(y); 
    
    W = sqrt(3)*(n(x)*G(1)-n(z)*E(1)); % intermediate variable
    
    % bf components
    B(y) = h(y)+((sqrt(W^2-3*lf^2*(1-n(y)^2)+(2*k(1))^2)-W)/2); 
    k(2) = B(y)-h(y); 
    B(x) = ((E(1)*k(2)-n(z)*T1)/k(1)) +h(x); 
    E(2) = B(x)-h(x); 
    B(z) = ((n(x)*T1+G(1)*k(2))/k(1))+h(z); 
    G(2) = B(z)-h(z); 
    
    % cf components
    c(y) = h(y)+((W+sqrt(W^2-3*lf^2*(1-n(y)^2)+(2*k(1))^2))/2); 
    k(3) = c(y)-h(y); 
    c(x) = ((E(1)*k(3)+n(z)*T1)/k(1)) +h(x); 
    E(3) = c(x)-h(x); 
    c(z) = ((G(1)*k(3)-n(x)*T1)/k(1))+h(z); 
    G(3) = c(z)-h(z); 
    
  
    %% STAGE 2 CALCULATIONS
    
    % a1
    a1f(x) = a(x)+(m/lf)*(n(z)*k(1)-n(y)*G(1)); 
    a1f(y) = a(y)+((a1f(x)-a(x))*k(1)-n(z)*lf*m)/E(1); 
    a1f(z) = a(z)+(n(y)*lf*m+(a1f(x)-a(x))*G(1))/E(1); 
    a1 = a1f - A10;
    
    % a2
    a2f(x) = 2*a(x)-a1f(x);
    a2f(y) = 2*a(y)-a1f(y); 
    a2f(z) = 2*a(z)-a1f(z); 
    a2 = a2f - A20; 
    
    % b1
    b1f(x) = B(x)+(m/lf)*(n(z)*k(2)-n(y)*G(2)); 
    b1f(y) = B(y)+((b1f(x)-B(x))*k(2)-n(z)*lf*m)/E(2); 
    b1f(z) = B(z)+(n(y)*lf*m+(b1f(x)-B(x))*G(2))/E(2); 
    b1 = b1f - B10; 
    
    % b2
    b2f(x) = 2*B(x)-b1f(x); 
    b2f(y) = 2*B(y)-b1f(y); 
    b2f(z) = 2*B(z)-b1f(z); 
    b2 = b2f - B20; 
    
    % c1
    c1f(x) = c(x)+(m/lf)*(n(z)*k(3)-n(y)*G(3)); 
    c1f(y) = c(y)+((c1f(x)-c(x))*k(3)-n(z)*lf*m)/E(3); 
    c1f(z) = c(z)+(n(y)*lf*m+(c1f(x)-c(x))*G(3))/E(3);
    c1 = c1f - C10; 
    
    % c2
    c2f(x) = 2*c(x)-c1f(x); 
    c2f(y) = 2*c(y)-c1f(y); 
    c2f(z) = 2*c(z)-c1f(z); 
    c2 = c2f - C20; 
    
    
    %% STAGE 3 CALCULATIONS
    
    % theta(1)
    a1s = sum(a1.*NOR_ac)*NOR_ac; % vector 'a1s'
    mag_a1s = sqrt(sum(a1s.^2)); % magnitude of vector 'a1s'
    a1_proj = a1 - a1s; % projection of vector 'a1' onto the ac plane
    mag_a1_proj = sqrt(sum(a1_proj.^2)); % magnitude of vector 'a1' projected on the ac plane
    mag_p2a1 = sqrt(p2^2-mag_a1s^2); % magnitude of link p2 projected on the ac plane
    theta(1) = acos(-sum(a1_proj.*AC)/(2*d2*mag_a1_proj)); % theta a1
    theta(1) = (theta(1) - acos((mag_a1_proj^2+p1^2-mag_p2a1^2)/(2*mag_a1_proj*p1)))*TOTAL_Deg; % theta a1 continued calculation
    
    % theta(2)
    a2s = sum(a2.*NOR_ab)*NOR_ab; 
    mag_a2s = sqrt(sum(a2s.^2)); 
    a2_proj = a2-a2s; 
    mag_a2_proj = sqrt(sum(a2_proj.^2)); 
    mag_p2a2 = sqrt(p2^2-mag_a2s^2); 
    theta(2) = acos(-sum(a2_proj.*AB)/(2*d2*mag_a2_proj)); 
    theta(2) = (theta(2) - acos((mag_a2_proj^2+p1^2-mag_p2a2^2)/(2*mag_a2_proj*p1)))*TOTAL_Deg; 
    
    % theta(3)
    b1s = sum(b1.*NOR_ab)*NOR_ab; 
    mag_b1s = sqrt(sum(b1s.^2)); 
    b1_proj = b1 - b1s;
    mag_b1_proj = sqrt(sum(b1_proj.^2)); 
    mag_p2b1 = sqrt(p2^2-mag_b1s^2); 
    theta(3) = acos(sum(b1_proj.*AB)/(2*d2*mag_b1_proj)); 
    theta(3) = (theta(3) - acos((mag_b1_proj^2+p1^2-mag_p2b1^2)/(2*mag_b1_proj*p1)))*TOTAL_Deg; 
    
    % theta(4)
    b2s = sum(b2.*NOR_bc)*NOR_bc; 
    mag_b2s = sqrt(sum(b2s.^2)); 
    b2_proj = b2 - b2s; 
    mag_b2_proj = sqrt(sum(b2_proj.^2)); 
    mag_p2b2 = sqrt(p2^2-mag_b2s^2); 
    theta(4) = acos(-sum(b2_proj.*BC)/(2*d2*mag_b2_proj)); 
    theta(4) = (theta(4) - acos((mag_b2_proj^2+p1^2-mag_p2b2^2)/(2*mag_b2_proj*p1)))*TOTAL_Deg; 
    
    % theta(5)
    c1s = sum(c1.*NOR_bc)*NOR_bc;
    mag_c1s = sqrt(sum(c1s.^2)); 
    c1_proj = c1 - c1s; 
    mag_c1_proj = sqrt(sum(c1_proj.^2)); 
    mag_p2c1 = sqrt(p2^2-mag_c1s^2); 
    theta(5) = acos(sum(c1_proj.*BC)/(2*d2*mag_c1_proj)); 
    theta(5) = (theta(5) - acos((mag_c1_proj^2+p1^2-mag_p2c1^2)/(2*mag_c1_proj*p1)))*TOTAL_Deg; 
    
    %theta(6)
    c2s = sum(c2.*NOR_ac)*NOR_ac; 
    mag_c2s = sqrt(sum(c2s.^2)); 
    c2_proj = c2 - c2s; 
    mag_c2_proj = sqrt(sum(c2_proj.^2)); 
    mag_p2c2 = sqrt(p2^2-mag_c2s^2); 
    theta(6) = acos(sum(c2_proj.*AC)/(2*d2*mag_c2_proj)); 
    theta(6) = (theta(6) - acos((mag_c2_proj^2+p1^2-mag_p2c2^2)/(2*mag_c2_proj*p1)))*TOTAL_Deg; 
    
    
    %% checking error
    check = 0;
    for i = 1:6
        if(abs(theta(i)) >= 60 || isnan(theta(i))) 
            check = 1;
        end
    end

    if(check == 1)
        fprintf("\nERROR: CURRENT VALUES CANNOT BE EXECUTED\n");
    else
        

    fprintf("theta(1) = %0.3f\n", theta(1));
    fprintf("theta(2) = %0.3f\n", theta(2));
    fprintf("theta(3) = %0.3f\n", theta(3));
    fprintf("theta(4) = %0.3f\n", theta(4));
    fprintf("theta(5) = %0.3f\n", theta(5));
    fprintf("theta(6) = %0.3f\n", theta(6));

end 