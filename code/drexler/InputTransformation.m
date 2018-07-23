function [control1, control2] = InputTransformation(velocity,angularVelocity,L,type)
%transforms linear and angular velocities of the kinematics car model to 
%the real inputs. there are three types of real car models 
% type == 1: steered front wheel, in this case the inputs and outputs are:
%   L: distance between the axes of the front and rear wheels
%   control1: the linear velocity of the car
%   control2: the wheel angle
% type == 2: steered rear wheel, in this case the inputs and outputs are:
%   L: distance between the axes of the front and rear wheels
%   control1: the linear velocity of the car
%   control2: the wheel angle
% type = 3: differentially driven car, in this case the inputs and outputs
%   are:
%   L: is a vector, L(1) is half of the distance between the wheels, L(2)
%      is the radius of the wheels
%   control1: the angular velocity of the left wheel
%   control2: the angular velocity of the right wheel 
switch type
    case 1 %front wheel steering
        control1 = velocity; %linear velocity
        control2 = atan2(angularVelocity*L,velocity); %wheel angle
    case 2 %back wheel steering (steer-drive)
        control1 = velocity; %linear velocity
        control2 = -atan2(angularVelocity*L,velocity); %wheel angle
    case 3 %differential drive
        D=L(1); %2*D is the distnance between the wheels
        r=L(2); %r is the raduis of the wheels
        control1 = (velocity-angularVelocity*D)/r; %left wheel velocity
        control2 = (velocity+angularVelocity*D)/r; %right wheel velocity
end