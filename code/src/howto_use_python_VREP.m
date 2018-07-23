%% This is an example to show how can the VREP API Facade can be used from matlab

% You need to create the objects, you wish to link in your Application
% Create the proper object for each actuator (for a velocity controlled one
% create a VelocityControleldJoint and etc. Make sure you pass the proper
% name of the object, otherwise it will not work!
frontMotor = py.ginop_vrep.VelocityControlledJoint('frontMotor');
steeringMotor = py.ginop_vrep.PositionControlledJoint('steeringMotor');
AGV_Center = py.ginop_vrep.DummyObject('AGV_Center');

% You need to pass the created objects as a list
joints = {frontMotor, steeringMotor, AGV_Center};
API = py.ginop_vrep.vrepCommunicationAPI(joints);
%% This part does the initialization & starts connection to the VREP
API.initialize()
API.startConnection()

%% This is just an example part, how to command the joints directly
for i = 1:1:100
    %Set the desired velocity for a velocity controlled joint
    frontMotor.setJointVelocity(sin(i / 10))
    %Set the target angle position controlled joint
    steeringMotor.setJointPosition(0)
    % Just showing some feedback, to make sure it works
    pos_list = AGV_Center.getObjectPosition();
    pos = cellfun(@double, cell(pos_list))
    % Since the Simulation is running in Syncronized mode, you need to call
    % this function to trigger the next iteration. This should be called
    % as the last step in the control loop.
    API.triggerStep()
end

%% Application finished, time to clean up the connection!
API.closeConnection()