function [kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize
    
    % kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper_KIMM.hrdf');
    kin = HEBI_Arm_Parameters('6dof_w_gripper'); % base�ٲ�鼭 ������
    gains = HebiUtils.loadGains('gains/6-DoF_arm_gains_KIMM[basic]_X8_9base.xml');
    trajGen = HebiTrajectoryGenerator(kin);

    familyName = 'Arm';%'6-DoF Arm';
    moduleNames = {'Base','Shoulder','Elbow','Wrist1','Wrist2','Wrist3'};

    group = HebiLookup.newGroupFromNames( familyName, moduleNames );
    group.send('gains',gains);
    cmd = CommandStruct();    
    
    % gripper setting
    grippergroup = HebiLookup.newGroupFromNames('Gripper','Spool');
    grippercmd = CommandStruct();
    
    group.setFeedbackFrequency(100);
    
end