function ReferenceTrajectory = GenerateBezier(P1,P2,P3,P4,SamplingTime,EndTime)

t = [0:SamplingTime:EndTime+SamplingTime];
t = t/t(end);

ReferencePosition = kron((1-t).^3,P1) + kron(3*(1-t).^2.*t,P2) + kron(3*(1-t).*t.^2,P3) + kron(t.^3,P4);
ReferenceOrientation = atan2(diff(ReferencePosition(2,:)),diff(ReferencePosition(1,:)));
ReferenceOrientation(end+1)=ReferenceOrientation(end); %the last two orientations are identical (this is just an approximation, but
% it is needed to avoid decreasing the number of samples)

ReferenceTrajectory= [ReferencePosition;ReferenceOrientation];