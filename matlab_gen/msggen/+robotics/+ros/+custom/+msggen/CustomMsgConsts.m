classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties (Constant)
        epos_tutorial_DesiredVel = 'epos_tutorial/DesiredVel'
        epos_tutorial_Vel = 'epos_tutorial/Vel'
        epos_tutorial_VelCommand = 'epos_tutorial/VelCommand'
        epos_tutorial_VelCommandRequest = 'epos_tutorial/VelCommandRequest'
        epos_tutorial_VelCommandResponse = 'epos_tutorial/VelCommandResponse'
        epos_tutorial_realVel = 'epos_tutorial/realVel'
        mobile_control_motorMsg = 'mobile_control/motorMsg'
        path_planner_des_traj = 'path_planner/des_traj'
        sim_control_cmdMsg = 'sim_control/cmdMsg'
        sim_control_desiredMsg = 'sim_control/desiredMsg'
        sim_control_groundMsg = 'sim_control/groundMsg'
        sim_control_motorDynamics = 'sim_control/motorDynamics'
        sim_control_slamposeMsg = 'sim_control/slamposeMsg'
        vehicle_control_commendMsg = 'vehicle_control/commendMsg'
        vehicle_control_gnd = 'vehicle_control/gnd'
        vehicle_control_jointstatesMsg = 'vehicle_control/jointstatesMsg'
        vehicle_control_motorsMsg = 'vehicle_control/motorsMsg'
        vehicle_control_noise = 'vehicle_control/noise'
        vehicle_control_posMsgs = 'vehicle_control/posMsgs'
        vehicle_control_positionMsg = 'vehicle_control/positionMsg'
        vehicle_control_time_loop = 'vehicle_control/time_loop'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(20, 1);
                msgList{1} = 'epos_tutorial/DesiredVel';
                msgList{2} = 'epos_tutorial/Vel';
                msgList{3} = 'epos_tutorial/VelCommandRequest';
                msgList{4} = 'epos_tutorial/VelCommandResponse';
                msgList{5} = 'epos_tutorial/realVel';
                msgList{6} = 'mobile_control/motorMsg';
                msgList{7} = 'path_planner/des_traj';
                msgList{8} = 'sim_control/cmdMsg';
                msgList{9} = 'sim_control/desiredMsg';
                msgList{10} = 'sim_control/groundMsg';
                msgList{11} = 'sim_control/motorDynamics';
                msgList{12} = 'sim_control/slamposeMsg';
                msgList{13} = 'vehicle_control/commendMsg';
                msgList{14} = 'vehicle_control/gnd';
                msgList{15} = 'vehicle_control/jointstatesMsg';
                msgList{16} = 'vehicle_control/motorsMsg';
                msgList{17} = 'vehicle_control/noise';
                msgList{18} = 'vehicle_control/posMsgs';
                msgList{19} = 'vehicle_control/positionMsg';
                msgList{20} = 'vehicle_control/time_loop';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(1, 1);
                svcList{1} = 'epos_tutorial/VelCommand';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
