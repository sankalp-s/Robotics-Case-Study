
clc
clear

% defines x, y, and z array indexes FIRST
[x, y, z] = deal(1, 2, 3);


%% PUT INPUT VALUES
N = [0.1, 0.1, 1]; % NORMAL vector
N = N/mag(N); % convert vector into unit vector

H = [0, 0, 4]; % center point of the platform
a(x) = 0.01; % x component of point 

% input user defined lengths
l0 = 2.875;
lf = 2.375;
d1 = 1;
d2 = 1.625;
m = 0.5;
p1 = 1;
p2 = 4.375;


%% MAIN CODE

while(1)
    First_choise = menu("Choose an Option ","Play Demo","Enter Values", "Exit");
    
    if(First_choise == 1)
        step = 0.15; 
        time = 0;
        
        for i = -0.2:step:0.2
         N = [i, 0, 1];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
        
        for i = 0.2:-step:0
         N = [i, 0, 1];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values 
         pause(time);
        end
        
        for i = 4:step:4.7
         H = [0, 0, i];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
        
        for i = 4.7:-step:4
         H = [0, 0, i];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values 
         pause(time);
        end
        
        for i = 0.01:step:1
         a(x) = i;
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values 
         pause(time);
        end
        
        for i = 1:-step:-1
         a(x) = i;
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values 
         pause(time);
        end
        
        for i = -1:step:0.01
         a(x) = i;
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values 
         pause(time);
        end
        
        for i = 0:step:1
         H = [i, 0, 4];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
        
        for i = 0:step:2*pi
         H = [cos(i), sin(i), 4];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
        
        for i = 1:-step:0
         H = [i, 0, 4];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
        
        for i = 0:step:2*pi
         N = [0.1*cos(i), 0.1*sin(i), 1];
         output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
         pause(time);
        end
    
    else if(First_choise == 2)
        Second_choise = menu("Choose an Option ","Change Normal Vector","Change Height Point", "Change Twist");
        if (Second_choise == 1)
            while(1)
                N(x) = input("\nEnter nx: ");
                if(N(x) == 444)
                    break
                end
                N(y) = input("Enter ny: ");
                if(N(y) == 444)
                    break
                end
                N(z) = input("Enter nz: ");
                if(N(z) == 444)
                    break
                end
                output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
            end
        end
        if (Second_choise == 2)
            while(1)
                H(x) = input("\nEnter hx: ");
                if(H(x) == 444)
                    break
                end
                H(y) = input("Enter hy: ");
                if(H(y) == 444)
                    break
                end
                H(z) = input("Enter hz: ");
                if(H(z) == 444)
                    break
                end
                output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
            end
        end
        
        if (Second_choise == 3)
            while(1)
                a(x) = input("\nEnter ax: ");
                if(a(x) == 444)
                    break
                end
                output_model(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcualtion of all theta values
            end
        end
    else
        break
    end
    end
end

