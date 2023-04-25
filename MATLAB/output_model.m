function output = output_model(n, h, ax, l0, lf, d1, d2, m, p1, p2) % calcuates all theta values
  
    % defines x, y, and z array indexes to be used
    [x, y, z] = deal(1, 2, 3);
    a(x) = ax;

   
    toDeg = 180/pi; % radians to degrees  
    
    %calculates normal vectors nab, nac, and nbc for each side of the triangle
    nab = [sqrt(3)/2, -1/2, 0];
    nac = [sqrt(3)/2, 1/2, 0];
    nbc = [0, 1, 0];
    
    % intermediate variables
    t = (lf^2*sqrt(3))/2;
    u = sqrt(l0^2+d1^2)*sin((2*pi/3)-atan(l0/d1)); %(2*pi/3 is 120 degrees)
    
    % calculates points a10, a20, b10, b20, c10, and c20
    a10 = [d2-u*sqrt(3), -u-d2*sqrt(3), 0]*0.5; 
    a20 = [-a10(x), a10(y), 0];
    b10 = [u*sqrt(3)+d2, d2*sqrt(3)-u, 0]*.5;
    b20 = [d2, u, 0];
    c10 = [-b20(x), b20(y), 0];
    c20 = [-b10(x), b10(y), 0];
    
    % calculates vectors, ab, ac, and bc
    ab = a20 - b10;
    ac = a10 - c20;
    bc = b20 - c10;
    
    %% ____________________
    %% CALCULATION OF STAGE FIRST
    
    
    % af components
    e(1) = a(x)-h(x); 
    a(z) = ((n(y)*sqrt(lf^2*(1-n(x)^2)-e(1)^2)-n(z)*n(x)*e(1))/(1-n(x)^2))+h(z); 
    g(1) = a(z)-h(z); 
    a(y) = h(y)-sqrt(lf^2-g(1)^2-e(1)^2); 
    k(1) = a(y)-h(y); 
    
    w = sqrt(3)*(n(x)*g(1)-n(z)*e(1)); 
    
    % bf components
    b(y) = h(y)+((sqrt(w^2-3*lf^2*(1-n(y)^2)+(2*k(1))^2)-w)/2); 
    k(2) = b(y)-h(y); 
    b(x) = ((e(1)*k(2)-n(z)*t)/k(1)) +h(x); 
    e(2) = b(x)-h(x); 
    b(z) = ((n(x)*t+g(1)*k(2))/k(1))+h(z); 
    g(2) = b(z)-h(z); 
    
    % cf components
    c(y) = h(y)+((w+sqrt(w^2-3*lf^2*(1-n(y)^2)+(2*k(1))^2))/2); 
    k(3) = c(y)-h(y); 
    c(x) = ((e(1)*k(3)+n(z)*t)/k(1)) +h(x); 
    e(3) = c(x)-h(x); 
    c(z) = ((g(1)*k(3)-n(x)*t)/k(1))+h(z); 
    g(3) = c(z)-h(z); 
    
    %% ____________________
    %% CALCULATION OF SECOND STAGE
    
    % a1
    a1f(x) = a(x)+(m/lf)*(n(z)*k(1)-n(y)*g(1)); 
    a1f(y) = a(y)+((a1f(x)-a(x))*k(1)-n(z)*lf*m)/e(1); 
    a1f(z) = a(z)+(n(y)*lf*m+(a1f(x)-a(x))*g(1))/e(1); 
    a1 = a1f - a10; 
    
    % a2
    a2f(x) = 2*a(x)-a1f(x); 
    a2f(y) = 2*a(y)-a1f(y); 
    a2f(z) = 2*a(z)-a1f(z); 
    a2 = a2f - a20; 
    
    % b1
    b1f(x) = b(x)+(m/lf)*(n(z)*k(2)-n(y)*g(2)); 
    b1f(y) = b(y)+((b1f(x)-b(x))*k(2)-n(z)*lf*m)/e(2); 
    b1f(z) = b(z)+(n(y)*lf*m+(b1f(x)-b(x))*g(2))/e(2); 
    b1 = b1f - b10; %
    
    % b2
    b2f(x) = 2*b(x)-b1f(x); 
    b2f(y) = 2*b(y)-b1f(y); 
    b2f(z) = 2*b(z)-b1f(z); 
    b2 = b2f - b20; 
    
    % c1
    c1f(x) = c(x)+(m/lf)*(n(z)*k(3)-n(y)*g(3)); 
    c1f(y) = c(y)+((c1f(x)-c(x))*k(3)-n(z)*lf*m)/e(3); 
    c1f(z) = c(z)+(n(y)*lf*m+(c1f(x)-c(x))*g(3))/e(3); 
    c1 = c1f - c10; 
    
    % c2
    c2f(x) = 2*c(x)-c1f(x); 
    c2f(y) = 2*c(y)-c1f(y); 
    c2f(z) = 2*c(z)-c1f(z); 
    c2 = c2f - c20; 
    
   
    %% CALCULATION OF THIRD STAGE
    
    % theta(1--)
    a1s = sum(a1.*nac)*nac; 
    mag_a1s = sqrt(sum(a1s.^2)); 
    a1_proj = a1 - a1s; 
    mag_a1_proj = sqrt(sum(a1_proj.^2)); 
    mag_p2a1 = sqrt(p2^2-mag_a1s^2); 
    theta(1) = acos(-sum(a1_proj.*ac)/(2*d2*mag_a1_proj)); 
    theta(1) = (theta(1) - acos((mag_a1_proj^2+p1^2-mag_p2a1^2)/(2*mag_a1_proj*p1)))*toDeg; 
    
    % theta(2)
    a2s = sum(a2.*nab)*nab; 
    mag_a2s = sqrt(sum(a2s.^2)); 
    a2_proj = a2-a2s; 
    mag_a2_proj = sqrt(sum(a2_proj.^2)); 
    mag_p2a2 = sqrt(p2^2-mag_a2s^2); 
    theta(2) = acos(-sum(a2_proj.*ab)/(2*d2*mag_a2_proj)); 
    theta(2) = (theta(2) - acos((mag_a2_proj^2+p1^2-mag_p2a2^2)/(2*mag_a2_proj*p1)))*toDeg; 
    
    % theta(3)
    b1s = sum(b1.*nab)*nab; 
    mag_b1s = sqrt(sum(b1s.^2)); 
    b1_proj = b1 - b1s; 
    mag_b1_proj = sqrt(sum(b1_proj.^2)); 
    mag_p2b1 = sqrt(p2^2-mag_b1s^2); 
    theta(3) = acos(sum(b1_proj.*ab)/(2*d2*mag_b1_proj)); 
    theta(3) = (theta(3) - acos((mag_b1_proj^2+p1^2-mag_p2b1^2)/(2*mag_b1_proj*p1)))*toDeg; 
    
    % theta(4)
    b2s = sum(b2.*nbc)*nbc; 
    mag_b2s = sqrt(sum(b2s.^2)); 
    b2_proj = b2 - b2s;
    mag_b2_proj = sqrt(sum(b2_proj.^2)); 
    mag_p2b2 = sqrt(p2^2-mag_b2s^2); 
    theta(4) = acos(-sum(b2_proj.*bc)/(2*d2*mag_b2_proj)); 
    theta(4) = (theta(4) - acos((mag_b2_proj^2+p1^2-mag_p2b2^2)/(2*mag_b2_proj*p1)))*toDeg; 
    
    % theta(5)
    c1s = sum(c1.*nbc)*nbc; 
    mag_c1s = sqrt(sum(c1s.^2)); 
    c1_proj = c1 - c1s; 
    mag_c1_proj = sqrt(sum(c1_proj.^2)); 
    mag_p2c1 = sqrt(p2^2-mag_c1s^2); 
    theta(5) = acos(sum(c1_proj.*bc)/(2*d2*mag_c1_proj)); 
    theta(5) = (theta(5) - acos((mag_c1_proj^2+p1^2-mag_p2c1^2)/(2*mag_c1_proj*p1)))*toDeg; 
    
    %theta(6)
    c2s = sum(c2.*nac)*nac; 
    mag_c2s = sqrt(sum(c2s.^2)); 
    c2_proj = c2 - c2s; 
    mag_c2_proj = sqrt(sum(c2_proj.^2)); 
    mag_p2c2 = sqrt(p2^2-mag_c2s^2); 
    theta(6) = acos(sum(c2_proj.*ac)/(2*d2*mag_c2_proj)); 
    theta(6) = (theta(6) - acos((mag_c2_proj^2+p1^2-mag_p2c2^2)/(2*mag_c2_proj*p1)))*toDeg; 
    
    
    %%  CHECKING ERROR
    check = 0;
    for i = 1:6
        if(abs(theta(i)) >= 60 || isnan(theta(i))) 
            check = 1;
        end
    end

    if(check == 1)
        fprintf("\nERROR: CURRENT VALUES CANNOT BE EXECUTED\n");
    else
       
        %% EXTRA CALCULATIONS (FOR MODEL)
        lmag = sqrt(sum(ab.^2)); 
        % a1m
        a1m(x) = a10(x)+(nac(y)*p1*lmag*cosd(theta(1)))/(nac(x)*ac(y)-nac(y)*ac(x));
        a1m(y) = ((nac(x)*a10(x)+nac(y)*a10(y))-nac(x)*a1m(x))/nac(y);
        a1m(z) = a1f(z)-sqrt(p2^2-(a1f(x)-a1m(x))^2-(a1f(y)-a1m(y))^2);
        % a2m
        a2m(x) = a20(x)+(nab(y)*p1*lmag*cosd(theta(2)))/(nab(x)*ab(y)-nab(y)*ab(x));
        a2m(y) = ((nab(x)*a20(x)+nab(y)*a20(y))-nab(x)*a2m(x))/nab(y);
        a2m(z) = a2f(z)-sqrt(p2^2-(a2f(x)-a2m(x))^2-(a2f(y)-a2m(y))^2);
        % b1m
        b1m(x) = b10(x)+(nab(y)*p1*lmag*cosd(theta(3)))/(nab(x)*-ab(y)-nab(y)*-ab(x));
        b1m(y) = ((nab(x)*b10(x)+nab(y)*b10(y))-nab(x)*b1m(x))/nab(y);
        b1m(z) = b1f(z)-sqrt(p2^2-(b1f(x)-b1m(x))^2-(b1f(y)-b1m(y))^2);
        % b2m
        b2m(x) = b20(x)+(nbc(y)*p1*lmag*cosd(theta(4)))/(nbc(x)*ab(y)-nbc(y)*bc(x));
        b2m(y) = ((nbc(x)*b20(x)+nbc(y)*b20(y))-nbc(x)*b2m(x))/nbc(y);
        b2m(z) = b2f(z)-sqrt(p2^2-(b2f(x)-b2m(x))^2-(b2f(y)-b2m(y))^2);
        % c1m
        c1m(x) = c10(x)+(nbc(y)*p1*lmag*cosd(theta(5)))/(nbc(x)*-bc(y)-nbc(y)*-bc(x));
        c1m(y) = ((nbc(x)*c10(x)+nbc(y)*c10(y))-nbc(x)*c1m(x))/nbc(y);
        c1m(z) = c1f(z)-sqrt(p2^2-(c1f(x)-c1m(x))^2-(c1f(y)-c1m(y))^2);
        % c2m
        c2m(x) = c20(x)+(nac(y)*p1*lmag*cosd(theta(6)))/(nac(x)*-ac(y)-nac(y)*-ac(x));
        c2m(y) = ((nac(x)*c20(x)+nac(y)*c20(y))-nac(x)*c2m(x))/nac(y);
        c2m(z) = c2f(z)-sqrt(p2^2-(c2f(x)-c2m(x))^2-(c2f(y)-c2m(y))^2);
        
      
        
        % base
        color1 = 'c';
        thick1 = 3; 
        side = l0*sqrt(3)-d1;
    
        quiver3(-d1, -l0, 0, d1*2, 0, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        hold on
        quiver3(d1, -l0, 0, 0.5*side, (sqrt(3)/2)*side, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        quiver3(-d1, -l0, 0, -0.5*side, sqrt(3)/2*side, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        quiver3(d1+0.5*side, ((sqrt(3)/2)*side)-l0, 0, -d1, sqrt(3)*d1, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        quiver3(-0.5*side-d1, ((sqrt(3)/2)*side)-l0, 0, d1, sqrt(3)*d1, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        quiver3(0.5*side, ((sqrt(3)/2)*side)-l0+d1*sqrt(3), 0, -side, 0, 0, 'ShowArrowHead', 'off', 'LineWidth', thick1, 'Color', color1, 'AutoScale', 'off');
        
        % ***link 1***
        thick2 = 3; 
        color2 = 'm';
        
        % link 1 a1 
        quiver3(a10(x), a10(y), a10(z), a1m(x)-a10(x), a1m(y)-a10(y), a1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        %link 1 a2
        quiver3(a20(x), a20(y), a20(z), a2m(x)-a20(x), a2m(y)-a20(y), a2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        %link 1 b1
        quiver3(b10(x), b10(y), b10(z), b1m(x)-b10(x), b1m(y)-b10(y), b1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        %link 1 b2
        quiver3(b20(x), b20(y), b20(z), b2m(x)-b20(x), b2m(y)-b20(y), b2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        %link 1 c1
        quiver3(c10(x), c10(y), c10(z), c1m(x)-c10(x), c1m(y)-c10(y), c1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        %link 1 c2
        quiver3(c20(x), c20(y), c20(z), c2m(x)-c20(x), c2m(y)-c20(y), c2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick2, 'Color', color2, 'AutoScale', 'off');
        
        % ***link 2***
        thick3 = 3; 
        color3 = 'g';
        
        % link 2 a1 
        quiver3(a1m(x), a1m(y), a1m(z), a1f(x)-a1m(x), a1f(y)-a1m(y), a1f(z)-a1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        % link 2 a2 
        quiver3(a2m(x), a2m(y), a2m(z), a2f(x)-a2m(x), a2f(y)-a2m(y), a2f(z)-a2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        % link 2 b1 
        quiver3(b1m(x), b1m(y), b1m(z), b1f(x)-b1m(x), b1f(y)-b1m(y), b1f(z)-b1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        % link 2 b2 
        quiver3(b2m(x), b2m(y), b2m(z), b2f(x)-b2m(x), b2f(y)-b2m(y), b2f(z)-b2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        % link 2 c1 
        quiver3(c1m(x), c1m(y), c1m(z), c1f(x)-c1m(x), c1f(y)-c1m(y), c1f(z)-c1m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        % link 2 c2 
        quiver3(c2m(x), c2m(y), c2m(z), c2f(x)-c2m(x), c2f(y)-c2m(y), c2f(z)-c2m(z), 'ShowArrowHead', 'off', 'LineWidth', thick3, 'Color', color3, 'AutoScale', 'off');
        
        % platform
        thick4 = 3; 
        color4 = 'b';
        
        quiver3(a1f(x), a1f(y), a1f(z), a2f(x)-a1f(x), a2f(y)-a1f(y), a2f(z)-a1f(z), 'ShowArrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        quiver3(a2f(x), a2f(y), a2f(z), b1f(x)-a2f(x), b1f(y)-a2f(y), b1f(z)-a2f(z), 'ShowArrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        quiver3(b1f(x), b1f(y), b1f(z), b2f(x)-b1f(x), b2f(y)-b1f(y), b2f(z)-b1f(z), 'ShowarrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        quiver3(b2f(x), b2f(y), b2f(z), c1f(x)-b2f(x), c1f(y)-b2f(y), c1f(z)-b2f(z), 'ShowArrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        quiver3(c1f(x), c1f(y), c1f(z), c2f(x)-c1f(x), c2f(y)-c1f(y), c2f(z)-c1f(z), 'ShowArrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        quiver3(c2f(x), c2f(y), c2f(z), a1f(x)-c2f(x), a1f(y)-c2f(y), a1f(z)-c2f(z), 'ShowArrowHead', 'off', 'LineWidth', thick4, 'Color', color4, 'AutoScale', 'off');
        
        hold off
    end
   
end