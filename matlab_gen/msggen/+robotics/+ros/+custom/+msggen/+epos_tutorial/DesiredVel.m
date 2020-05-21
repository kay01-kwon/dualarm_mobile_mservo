classdef DesiredVel < robotics.ros.Message
    %DesiredVel MATLAB implementation of epos_tutorial/DesiredVel
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'epos_tutorial/DesiredVel' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '83fd59d46058e7a3560e82aac6c5cc50' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Vel1
        Vel2
        Vel3
        Vel4
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Vel1', 'Vel2', 'Vel3', 'Vel4'} % List of non-constant message properties
        ROSPropertyList = {'vel1', 'vel2', 'vel3', 'vel4'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = DesiredVel(msg)
            %DesiredVel Construct the message object DesiredVel
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function vel1 = get.Vel1(obj)
            %get.Vel1 Get the value for property Vel1
            vel1 = int32(obj.JavaMessage.getVel1);
        end
        
        function set.Vel1(obj, vel1)
            %set.Vel1 Set the value for property Vel1
            validateattributes(vel1, {'numeric'}, {'nonempty', 'scalar'}, 'DesiredVel', 'Vel1');
            
            obj.JavaMessage.setVel1(vel1);
        end
        
        function vel2 = get.Vel2(obj)
            %get.Vel2 Get the value for property Vel2
            vel2 = int32(obj.JavaMessage.getVel2);
        end
        
        function set.Vel2(obj, vel2)
            %set.Vel2 Set the value for property Vel2
            validateattributes(vel2, {'numeric'}, {'nonempty', 'scalar'}, 'DesiredVel', 'Vel2');
            
            obj.JavaMessage.setVel2(vel2);
        end
        
        function vel3 = get.Vel3(obj)
            %get.Vel3 Get the value for property Vel3
            vel3 = int32(obj.JavaMessage.getVel3);
        end
        
        function set.Vel3(obj, vel3)
            %set.Vel3 Set the value for property Vel3
            validateattributes(vel3, {'numeric'}, {'nonempty', 'scalar'}, 'DesiredVel', 'Vel3');
            
            obj.JavaMessage.setVel3(vel3);
        end
        
        function vel4 = get.Vel4(obj)
            %get.Vel4 Get the value for property Vel4
            vel4 = int32(obj.JavaMessage.getVel4);
        end
        
        function set.Vel4(obj, vel4)
            %set.Vel4 Set the value for property Vel4
            validateattributes(vel4, {'numeric'}, {'nonempty', 'scalar'}, 'DesiredVel', 'Vel4');
            
            obj.JavaMessage.setVel4(vel4);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Vel1 = obj.Vel1;
            cpObj.Vel2 = obj.Vel2;
            cpObj.Vel3 = obj.Vel3;
            cpObj.Vel4 = obj.Vel4;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Vel1 = strObj.Vel1;
            obj.Vel2 = strObj.Vel2;
            obj.Vel3 = strObj.Vel3;
            obj.Vel4 = strObj.Vel4;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Vel1 = obj.Vel1;
            strObj.Vel2 = obj.Vel2;
            strObj.Vel3 = obj.Vel3;
            strObj.Vel4 = obj.Vel4;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.epos_tutorial.DesiredVel.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.epos_tutorial.DesiredVel;
            obj.reload(strObj);
        end
    end
end