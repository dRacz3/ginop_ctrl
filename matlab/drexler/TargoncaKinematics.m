function dXVec = TargoncaKinematics(t,XVec,LinearVelocity,angularVelocity)

x = XVec(1);
y = XVec(2);
theta = XVec(3);

dx = cos(theta)*LinearVelocity; 
dy = sin(theta)*LinearVelocity;
dtheta = angularVelocity;

dXVec = [dx;dy;dtheta];