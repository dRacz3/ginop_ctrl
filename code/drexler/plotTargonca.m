function plotTargonca(fig,position,orientation,type,scale,color,wheelOrientation)

BodyX = [-1 3 3 -1 -1];
BodyY = [2 2 -2 -2 2];
leftWheelX = [-0.75 -0.75 0.25 0.25 -0.75 ];
leftWheelY = [2 2.25 2.25 2 2];
rightWheelX = [-0.75 -0.75 0.25 0.25 -0.75 ];
rightWheelY = -[2 2.25 2.25 2 2];
if type==1
rotatedWheelX0 = [ 2.5 3.5 3.5 2.5 2.5]-3;
rotatedWheelY0 = [ 1/6 1/6 -1/6 -1/6 1/6];
rotatedWheel = planarRot(wheelOrientation)*[rotatedWheelX0;rotatedWheelY0];
rotatedWheel(1,:)=rotatedWheel(1,:)+2.5;
end
leftForkX = [3 5 5 3 3];
leftForkY = [1.25 1.25 1 1 1.25];
rightForkX = [3 5 5 3 3];
rightForkY = -[1.25 1.25 1 1 1.25];
Body = planarTransform(orientation,position,scale)*[BodyX;BodyY;ones(1,length(BodyX))];
leftWheel = planarTransform(orientation,position,scale)*[leftWheelX;leftWheelY;ones(1,length(leftWheelX))];
rightWheel = planarTransform(orientation,position,scale)*[rightWheelX;rightWheelY;ones(1,length(rightWheelX))];
if type==1
rotatedWheel = planarTransform(orientation,position,scale)*[rotatedWheel(1,:);rotatedWheel(2,:);ones(1,length(rotatedWheelX0))];
end
leftFork = planarTransform(orientation,position,scale)*[leftForkX;leftForkY;ones(1,length(leftForkX))];
rightFork = planarTransform(orientation,position,scale)*[rightForkX;rightForkY;ones(1,length(rightForkX))];

figure(fig)
plot(Body(1,:),Body(2,:),color,'LineWidth',2);
hold on
fill(leftWheel(1,:),leftWheel(2,:),color);
fill(rightWheel(1,:),rightWheel(2,:),color);
if type==1
fill(rotatedWheel(1,:),rotatedWheel(2,:),color);
end
fill(leftFork(1,:),leftFork(2,:),color);
fill(rightFork(1,:),rightFork(2,:),color);
hold off
end