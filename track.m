clc, clear, close all;

%路徑規劃
sampT = 0.001;
x = zeros(1,10/sampT);
y = zeros(1,10/sampT);
theta1 = zeros(1,10/sampT);
theta2 = zeros(1,10/sampT);
dtheta1 = zeros(1,10/sampT);
dtheta2 = zeros(1,10/sampT);
ddtheta1 = zeros(1,10/sampT);
ddtheta2 = zeros(1,10/sampT);
Xdes1 = [-0.05, 0, 0, 0.25, 0, 0.25];
Ydes1 = [0.1, 0.12, 0, 0, 0, 0];
Xdes2 = [0, 0.05, 0.25, 0, 0.25, 0];
Ydes2 = [0.12, 0.15, 0, 0, 0, 0];
a1 = Xdes1*inv(Tcom(0,5));
b1 = Ydes1*inv(Tcom(0,5));
a2 = Xdes2*inv(Tcom(5,10));
b2 = Ydes2*inv(Tcom(5,10));
i = 1;
for t = 0:sampT:10
    if(t>=0 && t<=5)
        x(i) = a1*T_matrix(t);
        y(i) = b1*T_matrix(t);
    elseif(t>5 && t<=10)
        x(i) = a2*T_matrix(t);
        y(i) = b2*T_matrix(t);
    end
    i = i+1;
    if( i > 10/sampT)
        break;
    end
end

%plot(x,y)
%xlabel('X-axis')
%ylabel('Y-axis')

%% 逆向運動學
j = 1;
l1 = 0.24;
l2 = 0.24;
for t=0:sampT:10
    theta2(j) = acos((x(j)^2 + y(j)^2 - l1^2 - l2^2)/(2*l1*l2));
    theta1(j) = atan(((l1+l2*cos(theta2(j)))*y(j)-l2*sin(theta2(j))*x(j))...
       /((l1+l2*cos(theta2(j)))*x(j)+l2*sin(theta2(j))*y(j)));
   j = j+1;
   if( j > 10/sampT)
       break;
   end
end

%V,A
j = 2;
for t=0:sampT:10
   dtheta1(j) = (theta2(j) - theta2(j-1))/sampT;
   dtheta2(j) = (theta2(j) - theta2(j-1))/sampT;
   ddtheta1(j) = (dtheta2(j) - dtheta2(j-1))/sampT;
   ddtheta2(j) = (dtheta2(j) - dtheta2(j-1))/sampT;
   j = j+1;
   if( j > 10/sampT)
       break;
   end
end


t=0:sampT:10-sampT;
plot(t, theta1)
hold on
plot(t, theta2)
xlabel('time(s)')
ylabel('Angle(rad)')

fid = fopen('axis1.txt','wt');
i=1;
for t = 0:sampT:10-sampT
    fprintf(fid,'%f\t',theta1(i));
    fprintf(fid,'%f\t',dtheta1(i));
    fprintf(fid,'%f\n',ddtheta1(i));
    i = i+1;
    if(i==10/sampT)
        break;
    end
end

fid = fopen('axis2.txt','wt');
i=1;
for t = 0:sampT:10-sampT
    fprintf(fid,'%f\t',theta2(i));
    fprintf(fid,'%f\t',dtheta2(i));
    fprintf(fid,'%f\n',ddtheta2(i));
    i = i+1;
    if(i==10/sampT)
        break;
    end
end
fclose(fid);
   
 fid = fopen('xy.txt','wt');
i=1;
for t = 0:sampT:10-sampT
    fprintf(fid,'%f\t',x(i));
    fprintf(fid,'%f\n',y(i));
    i = i+1;
    if(i==10/sampT)
        break;
    end
end
fclose(fid);   
    
    
    
    
    
    
    
    
    