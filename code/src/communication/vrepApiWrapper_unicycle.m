classdef vrepApiWrapper_unicycle < handle
    properties
        %Remote api object , functions are wrapped around this
        vrep;
        clientID; % Session ID

        frontMotor
        rightMotor;
        steeringMotor;
        
        center;

        jointVelocityParamID = 2012;
    end

    methods
        function self = vrepApiWrapper_unicycle()
          self.vrep = remApi('remoteApi');
        end

        %start
        function startConnection(self, ip, port)
          self.vrep.simxFinish(-1); % just in case, close all opened connections
          if nargin == 3
            self.clientID = self.vrep.simxStart(ip,port,true,false,1000,5);
          else
            display('Using default connection localhost:19997');
            self.clientID = self.vrep.simxStart('127.0.0.1',19997,true,false,1000,5);
          end

          if (self.clientID == -1) % It means that connection was not successful
               display('Connection failed');
          else
               display('Connection successful');
              self.vrep.simxSynchronous(self.clientID,true);
              self.vrep.simxStartSimulation(self.clientID,self.vrep.simx_opmode_oneshot_wait);

              self.getObjectHandles(self.clientID);
              self.initializeMotors(self.clientID);
          end
        end

        %Stop the simulation
        function closeConnection(self)
          self.vrep.simxStopSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait);
        end

        %Trigger simulation step
        function triggerStep(self)
          self.vrep.simxSynchronousTrigger(self.clientID);
        end

        function getObjectHandles(self, clientID)
            [error,self.frontMotor]     = self.vrep.simxGetObjectHandle(clientID,'frontMotor'    ,self.vrep.simx_opmode_oneshot_wait);
            [error,self.steeringMotor] = self.vrep.simxGetObjectHandle(clientID,'steeringMotor',self.vrep.simx_opmode_oneshot_wait);
            [error,self.center]     = self.vrep.simxGetObjectHandle(clientID,'AGV_Center',self.vrep.simx_opmode_oneshot_wait);
        end

        function initializeMotors(self, clientID)
          [returnCode] = self.vrep.simxSetJointTargetVelocity(clientID, self.frontMotor,     0     , self.vrep.simx_opmode_oneshot_wait);
          [returnCode] = self.vrep.simxSetJointTargetPosition(clientID, self.steeringMotor, 0     , self.vrep.simx_opmode_oneshot_wait);
          [returnCode, position] = self.vrep.simxGetJointPosition(clientID, self.frontMotor        , self.vrep.simx_opmode_streaming);
          [returnCode, position] = self.vrep.simxGetJointPosition(clientID, self.steeringMotor    , self.vrep.simx_opmode_streaming);
          [returnCode, velocity] = self.vrep.simxGetObjectFloatParameter(clientID, self.frontMotor , self.jointVelocityParamID, self.vrep.simx_opmode_streaming);
        end

         
        %This function returns the x,y,z position of the AGV Center dummy
        function position = getPosition(self)
            [return_code, position] = self.vrep.simxGetObjectPosition(self.clientID, self.center, -1, self.vrep.simx_opmode_oneshot_wait);
        end
        
        %This function returns the Euler angles of the AGV Center dummy
        function orientation = getOrientation(self)
            [return_code, orientation] = self.vrep.simxGetObjectOrientation(self.clientID, self.center, -1, self.vrep.simx_opmode_oneshot_wait);    
        end
        
        % Simplified API methods to handle motors easily:
        function vel = getMotorVelocities(self)
          vel = self.getJointVelocity(self.frontMotor);
        end

        function setMotorVelocities(self, vel)
          self.setJointVeloStream(self.frontMotor , vel);
        end

        %
        function steeringAngle = getSteeringAngle(self)
          [returnCode, steeringAngle] = self.getJointPositionBuffer(self.steeringMotor);
          self.handleReturnValue(returnCode);
        end

        function setSteeringAngleTarget(self, targetAngle)
          self.setJointTargetPositionStream(self.steeringMotor, targetAngle);
        end
        

       function setJointVeloStream(self,ObjectHandle,TargetVelocity)
       % setJointVeloOneShot  set Joint velocity repeatedly
       %   Call arguments: (ObjectHandle , TargetVelocity)
           returnCode = self.vrep.simxSetJointTargetVelocity(self.clientID,ObjectHandle,TargetVelocity,self.vrep.simx_opmode_streaming);
           self.handleReturnValue(returnCode);
       end

       % Set the target position for the joint ( streaming )
       function setJointTargetPositionStream(self, ObjectHandle, TargetPosition)
         returnCode = self.vrep.simxSetJointTargetPosition(self.clientID, ObjectHandle, TargetPosition, self.vrep.simx_opmode_oneshot_wait);
         self.handleReturnValue(returnCode);
       end

       function velocity = getJointVelocity(self, ObjectHandle)
        % clientID: the client ID. refer to simxStart.
        % objectHandle: handle of the object
        % parameterID: identifier of the parameter to retrieve. See the list of all possible object parameter identifiers
        % parameterValue: pointer to a location that will receive the value of the parameter
        % operationMode: a remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls), or simx_opmode_blocking (depending on the intended usage)
          [returnCode, velocity] = self.vrep.simxGetObjectFloatParameter(self.clientID, ObjectHandle, self.jointVelocityParamID, self.vrep.simx_opmode_buffer);
          self.handleReturnValue(returnCode);
       end

       function [returnCode,position] = getJointPositionBuffer(self,ObjectHandle)
           % getJointPositionBuffer gets the Passed handle's joint
           % position, must call "getJointPositionFirstCall" first
           [returnCode, position] = self.vrep.simxGetJointPosition(self.clientID, ObjectHandle, self.vrep.simx_opmode_buffer);
           self.handleReturnValue(returnCode);
       end

       function [returnCode, linearVelocity, angularVelocity] = getObjectVelocityFirstCall(self,ObjectHandle)
           % getObjectVelocityFirstCall gets the Passed handle's
           % velocity, must call this before "getObjectVelocityBuffer"
           [returnCode, linearVelocity, angularVelocity] = self.vrep.simxGetObjectVelocity(self.clientID,ObjectHandle,self.vrep.simx_opmode_streaming);
       end
       function [returnCode, linearVelocity, angularVelocity] = getObjectVelocityBuffer(self,ObjectHandle)
           % getObjectVelocityBuffer gets the Passed handle's
           % velocity, must call "getObjectVelocityFirstCall" first
           [returnCode, linearVelocity, angularVelocity] = self.vrep.simxGetObjectVelocity(self.clientID,ObjectHandle,simx_opmode_buffer);
       end

       function handleReturnValue(self, returnValue)
         if returnValue == 0
           % This is fine
           return
         elseif returnValue == 1
           %display('There is no command reply in the input buffer -> Not always an error, depending on the mode its okay')
         elseif returnValue == 2
           display('The function timed out ')
         elseif returnValue == 4
           display('The specified operation mode is not supported for the given function')
         elseif returnValue == 8
           display('The function caused an error on the server side (e.g. an invalid handle was specified)')
         elseif returnValue == 16
           display('The communication thread is still processing previous split command of the same type')
         elseif returnValue == 32
           display('The function caused an error on the client side')
         elseif returnValue == 64
           display('simxStart was not yet called')
         end
       end
    end
end
