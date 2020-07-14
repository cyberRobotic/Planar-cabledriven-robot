clear all;
close all;
clc;

% Triangel side
l_b=3;

% Postion triangle end points
A_1=[0,0];
A_2=[l_b,0];
A_3=[l_b/2,(sqrt(3)*l_b)/2];
A=[A_1;A_2;A_3];

% Position
x=1.5;
y=1.0;

% Backward kinematics
cables=backwardKinematics(A,x,y);
theta=angle(A,x,y);

% Forward kinematics
pos=forwardKinematics(cables,l_b);

% Plot
plotBaseArea(A);
plotPlatform(x,y);
plotCables(A,cables,theta);


function plotBaseArea(A)
for i=1:length(A)
    plot(A(i,1),A(i,2),'-o')
    hold on
end
for i=1:length(A)
    if(i==3)
        line([A(i,1),A(i-2,1)],[A(i,2),A(i-2,2)]);
        hold on
        break;
    end
    line([A(i,1),A(i+1,1)],[A(i,2),A(i+1,2)]);
    hold on
    
end
end

function plotPlatform(x,y)
plot(x,y,'-o')
hold on;
end

function plotCables(A,l,theta)
    line([A(1,1),A(1,1)+l(1)*cos(theta(1))],[A(1,2),A(1,2)+l(1)*sin(theta(1))],'color','red');
    hold on
    line([A(2,1),A(2,1)-l(2)*cos(theta(2))],[A(2,2),A(2,2)-l(2)*sin(theta(2))],'color','red');
    hold on
    if(A(3,1)+l(3)*cos(theta(3))>A(3,1))
        line([A(3,1),A(3,1)-l(3)*cos(theta(3))],[A(3,2),A(3,2)-l(3)*sin(theta(3))],'color','red');
    else
        line([A(3,1),A(3,1)-l(3)*cos(theta(3))],[A(3,2),A(3,2)-l(3)*sin(theta(3))],'color','red');
    end
    hold on
end

function l=backwardKinematics(A,x,y)
    l_1=0;
    l_2=0;
    l_3=0;
    l=[l_1; l_2; l_3;];
    for i=1:length(A)
        l(i)=sqrt((x-A(i,1))^2+(y-A(i,2))^2);
    end
end

function theta=angle(A,x,y)
    theta_1=0;
    theta_2=0;
    theta_3=0;
    theta=[theta_1;theta_2;theta_3];
    for i=1:length(A)
        theta(i)=(atan((y-A(i,2))/(x-A(i,1))));
        if(x-A(i,1))==0
            theta(i)=pi/2;
        end
    end
end

function pos=forwardKinematics(l,l_b)
    x=(l_b^2+l(1)^2-l(2)^2)/(2*l_b);
    y=sqrt(l(1)^2-x^2);
    pos=[x,y];
    disp(y==y)
end

