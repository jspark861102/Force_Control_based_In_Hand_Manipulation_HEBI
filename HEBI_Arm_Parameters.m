function [kin] = HEBI_Arm_Parameters(kit)

%% Setup kinematic models
switch kit
    
    case '6dof_w_gripper' % KIMM
        %%        
        % Approximate kinematics to the tip of the gripper, expressed in the
        % output frame of the last module on the arm
        gripperOutput = eye(4);
        gripperOutput(1:3,4) = [0; 0; .075];

        % Kinematic Model
        % 6-DoF Arm w/ Gripper
        kin = HebiKinematics();
%         kin.addBody('X5-9'); %���ݱ��� ��Ծ���, �����δ� X5-9�ε� �߸����� ���� ���߿� �����ʿ�
%         kin.addBody('X8-3');
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-inside');
        kin.addBody('X8-16'); 
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi, ...
                        'mass', .500); % Added mass for the gripper spool
        kin.addBody('X8-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody( 'GenericLink', 'CoM', [0 0 .025], ...
                                    'Output', gripperOutput, ...
                                    'Mass', .100 );
      
        % Torques for the girpper spool to open-close the gripper
%         params.gripperOpenEffort = 1;
%         params.gripperCloseEffort = -5;
    end


end

