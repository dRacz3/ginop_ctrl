function ReferenceInput = GenerateReferenceInput(ReferenceTrajectory,SamplingTime)

XLIMIT = 0.00001;

N = size(ReferenceTrajectory,2);
for ii=1:N-1
    if ii==1
        phik = ReferenceTrajectory(3,ii); %phi,ref,k    
    else
        phik = ReferenceTrajectory(3,ii-1); %phi,ref,k
    end
    xk = ReferenceTrajectory(1,ii); %x,ref,k spatial
    yk = ReferenceTrajectory(2,ii); %y,ref,k spatial
    xs = ReferenceTrajectory(1,ii+1); %x,ref,k+1 spatial
    ys = ReferenceTrajectory(2,ii+1); %y,ref,k+1 spatial
    DBody = inv(planarTransform(phik,[xk;yk],1))*[xs;ys;1];
    xb = DBody(1); %x,ref,k+1 body
    yb = DBody(2); %y,,ref,k+1 body
   
    if abs(xb) < XLIMIT %the desired motion is in the direction of the rear wheel axis
        RefVel = [0;0];
    else 
        RefVel = 1/(SamplingTime*xb)*[xb*cos(phik)-yb*sin(phik) xb*sin(phik)+yb*cos(phik); -sin(phik) cos(phik)]*[xs-xk;ys-yk];
    end

ReferenceInput(:,ii) = RefVel;

end
% ReferenceInput(2,:) = -ReferenceInput(2,:);
end