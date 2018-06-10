%Main script
clear all
close all
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                           Constants                              %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%front and rear axis distance of the robot

type = 1;
L=0.4;
%type = 2;
%L=2;
%type = 3;
%L=[0.5;0.15]

API = vrepApiWrapper_unicycle;
API.startConnection('127.0.0.1', 19997);

EndTime = 20; %sec %final time of the simulation
PauseTime = 0.1; %sec %delay between plots
SamplingTime = 0.100; %sec %sampling time

XMin = -10; % x axis plot minimum value
XMax = 30; % x axis plot maximum value
YMin = -10; % y axis plot minimal value
YMax = 30; % y axis plot maximal value
MaxVelocity = 10; %maximal linear velocity of the car, m/s


%initialize the plot
fig = figure(1) %first figure will show the trajetory and the robot
plot(0,0,'.w') %dummy plot for setting the axis limits
axis([XMin XMax YMin YMax])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                   Trajcetory generation                         %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Bezier curve
% title('Give the initial position of the desired path')
% P1  = ginput(1)';
% axis([XMin XMax YMin YMax])
% hold on
% plot(P1(1),P1(2),'rx','Linewidth',2)
% 
% title('Give the initial tangent of the desired path')
% P2 = ginput(1)';
% plot([P1(1) P2(1)],[P1(2) P2(2)],'b')
% 
% 
% title('Give the final position of the desired path')
% P4 = ginput(1)';
% plot(P4(1),P4(2),'rx','Linewidth',2)
% 
% title('Give the final tangent of the desired path')
% P3 = ginput(1)';
% plot([P3(1) P4(1)],[P3(2) P4(2)],'b')
% 
% ReferenceTrajectory = GenerateBezier(P1,P2,P3,P4,SamplingTime,EndTime);

% %circle
tvec = [0:SamplingTime:EndTime+SamplingTime]/(SamplingTime+EndTime)*2*pi;
ReferenceTrajectory = [1+5*cos(tvec);1+5*sin(tvec);tvec+pi/2];

plot(ReferenceTrajectory(1,:),ReferenceTrajectory(2,:),'r','Linewidth',1.5)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%               Reference velocities generation                   %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

title('Generating the reference velocities, please wait...')
ReferenceInputs = GenerateReferenceInput(ReferenceTrajectory,SamplingTime);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                  Initial pose of the robot                       %%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

title('Give the initial position of the car')

% InitialPosition = ginput(1)';
InitialPosition=ReferenceTrajectory(1:2,1); %no initial position error
plot(InitialPosition(1),InitialPosition(2),'rd','Linewidth',2)

title('Give the initial heading of the car')
% InitialHeading = ginput(1)';
% InitialOrientation = atan2(InitialHeading(2)-InitialPosition(2),InitialHeading(1)-InitialPosition(1));
InitialOrientation=ReferenceTrajectory(3,1); % no initial orientation
% error
InitialWheelOrientation = 0;
plotTargonca(fig,InitialPosition,InitialOrientation,InitialWheelOrientation,1,'b');

title('Press any key to start')
pause
saveas(gcf,'fig0','fig')
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                 Simulation, control and plot                     %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

oldX = [InitialPosition;InitialOrientation];
X = [];
RealInputs = [];
RefTVec = [];
RefIVec = [];
timeVec = [];
ii = 1;
for t = [0:SamplingTime:EndTime]
    RefVelocity = ReferenceInputs(1,ii); %the current reference linear velocity (v_d)
    RefAngularVelocity = ReferenceInputs(2,ii); % the current reference angular velocity (\omega_d)
    
   %trajectory tracking control 
   [ velocity,angularVelocity ] = TrackingControl(oldX, RefVelocity,RefAngularVelocity, ReferenceTrajectory(:,ii));
    if velocity > MaxVelocity %saturaton on the linear velocity
        velocity = MaxVelocity;
    end
    
    [control1,control2]=InputTransformation(velocity,angularVelocity,L,type);
    
    %%%%% Set the velocity to VREP robot %%%%%
    %For the unicycle model, "leftMotor" has been placed to the front
    %0.15 * 0.5 is the radius of the wheels
    API.setMotorVelocities(control1 / (0.5 * 0.15));
    %Set the steering angle target for AGV
    API.setSteeringAngleTarget(control2);
    position = API.getPosition();
    orientation = API.getOrientation()
    %Trigger a simulation step
    API.triggerStep();

    simvel = API.getJointPositionBuffer(API.frontMotor);
    % simulation for a time interval of [0,SamplingTime]
    [time,newX] = ode45(@(time,newX) TargoncaKinematics(time,newX,velocity,angularVelocity),[0 SamplingTime],oldX);
    time = time';
    %storage of the variables for plot and plotting the robot
    if length(timeVec) > 0 %add the current time (ii*SamplingTime) to simulation time that is always between 0 and SamplingTime
        timeVec = [timeVec time+timeVec(end)];
    else
        timeVec = time;
    end
    RefTVec = [RefTVec [ReferenceTrajectory(1,ii)*ones(size(time));ReferenceTrajectory(2,ii)*ones(size(time));ReferenceTrajectory(3,ii)*ones(size(time))]];
    RefIVec = [RefIVec [ReferenceInputs(1,ii)*ones(size(time));atan2(ReferenceInputs(2,ii)*L,ReferenceInputs(1,ii))*ones(size(time))]];
    RealInputs = [RealInputs [control1*ones(size(time));control2*ones(size(time))]];
    X = [X newX'];
    newX = newX(end,:)';
    oldX = newX;
    plotTargonca(fig,newX(1:2),newX(3),type,1,'b',control2);
    hold on
    plot(ReferenceTrajectory(1,:),ReferenceTrajectory(2,:),'g')
    plot(ReferenceTrajectory(1,ii),ReferenceTrajectory(2,ii),'rx')
    hold off
    axis([XMin XMax YMin YMax])
    if type==3
        text(XMax-6,YMax-5,sprintf(['t=' num2str(t) '\n omega_1=' num2str(control1) '\n omega_2=' num2str(control2)]))
    else
        text(XMax-6,YMax-5,sprintf(['t=' num2str(t) '\nv=' num2str(control1) '\ntheta=' num2str(control2)]))
    end
    pause(PauseTime)
    if t==0
        saveas(gcf,'fig1','fig')
    elseif abs(t-EndTime/3) < SamplingTime    
        saveas(gcf,'fig2','fig')
    elseif abs(t-2*EndTime/3) <SamplingTime
        saveas(gcf,'fig3','fig')
    elseif t== EndTime
        saveas(gcf,'fig4','fig')
    end
    ii=ii+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%            Plots of the trajectories and inputs                   %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
figure(2)
subplot(3,1,1)

plot(timeVec,RefTVec(1,:),'r',timeVec,X(1,:),'g--','Linewidth',2)
title('x spatial coordinates')
legend('Reference','Realized')
xlabel('Time [sec]')
ylabel('x cordinate [m]')

subplot(3,1,2)

plot(timeVec,RefTVec(2,:),'r',timeVec,X(2,:),'g--','Linewidth',2)
title('y spatial coordinates')
legend('Reference','Realized')
xlabel('Time [sec]')
ylabel('y cordinate [m]')

subplot(3,1,3)

plot(timeVec,RefTVec(3,:),'r',timeVec,X(3,:),'g--','Linewidth',2)
title('Orientation')
legend('Reference','Realized')
xlabel('Time [sec]')
ylabel('orientation [rad]')

pause(2*PauseTime)
figure(3)

subplot(2,1,1)

plot(timeVec,RefIVec(1,:),'r',timeVec,RealInputs(1,:),'g--','Linewidth',2)
title('linear velocity')
legend('Reference','Realized')
xlabel('Time [sec]')
ylabel('linear velocity [m/s]')

subplot(2,1,2)

plot(timeVec,RefIVec(2,:),'r',timeVec,RealInputs(2,:),'g--','Linewidth',2)
title('steering angle')
legend('Reference','Realized')
xlabel('Time [sec]')
ylabel('steering angle [rad]')
