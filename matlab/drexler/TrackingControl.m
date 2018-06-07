function [velocity,angularVelocity] = TrackingControl(OldX, RefVelocity,RefAngularVelocity, ReferenceTrajectory)
%calculates the linear and angular velocities of the kinematic car model
%the inputs are:
%   OldX: pose of the car in the spatial frame, OldX(1) is the
%     x-coordinate, OldX(2) is the y-coordinate, OldX(3) is the
%     orientation, givein in the kth step
%   RefVelocity: the reference linear velocity in the kth step
%   RefAngularVelocity: the reference angular velocity in the kth step
%   ReferenceTrajectory: the reference trajectory given in the spatial
%    frame, for the (k+1)th step, ReferenceTrajectory(1) is the
%    x-coordinate, ReferenceTrajectory(2) is the y-coordinate,
%    ReferenceTrajectory(3) is the orientation


%%feedback gains
k1 = 0.5;
k2 = 0.5;
k3 = 1;
% for testing if the reference velocities are correct
% k1=0;
% k2=0;
% k3=0;

xk = OldX(1); %x,k spatial
yk = OldX(2); %y,k spatial
phik = OldX(3); %phi,k spatial

xr = ReferenceTrajectory(1); %x,ref,k+1 spatial
yr = ReferenceTrajectory(2); %x,ref,k+1 spatial
phir = ReferenceTrajectory(3); %phi,ref,k+1 spatial

ek = [xk;yk]-[xr;yr]; %tracking error, spatial
eref = planarRot(phir)\ek; %tracking error, reference
erx = eref(1); %e,x reference
ery = eref(2); %e,y reference
orib = mod(mod(phik,2*pi)-mod(phir,2*pi),2*pi); %orientation error, reference

if k1==0 %for testing if the reference velocities are correct
    velocity = RefVelocity;
else %the actual feedback control
velocity = (RefVelocity-k1*abs(RefVelocity)*(erx+ery*cos(orib)))/cos(orib);
end
angularVelocity = RefAngularVelocity-(k2*RefVelocity*ery+k3*abs(RefVelocity)*tan(orib))*cos(orib)^2;
end