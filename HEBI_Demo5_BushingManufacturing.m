%������ ���ۿ� ���� ������ Ȯ��
%�Ǻ����� ��� ����?

% �ν� ���� ���� ����
% ������� �ٽ� ��ġ����� ���ư��� ���۷��� ������ �ذ���
% Z���� ��� ���� ������ ������ ��ġ ��Ȯ�� �� ������ ������ �� ���ڿ������� ���� ����(Z���� ��� �߷°� ���� �������� �������� �������� �ʰ� ���ִ� ��� ����)
%%
clear *;
close all;
clc

%% setting parmaeters
%11:pick&insert
%12:pick&pushing&pivoting&place
%13:pick&pushing&insert
demo_case = 13;

%controlmode = 1 : cartesian space control
%controlmode = 2 : joint pace control
controlmode = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

% position/force control Threshold
desired_force = [1000 -13 1000 1000 1000 1000]; %������ ���� �ϰ� ���� ���� ���� ���� ũ�� ��������

% force control smooting
smoothing_duration = 0.2; % sec

% holdeffect velocity Threshold
velocityThreshold = 1;

%% HEBI setting
HebiLookup.initialize();
[kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;
group.startLog('dir','logs');
           
%% Target Waypoints          
[xyzTargets, rotMatTarget, control_time, gripperforce] = TargetWaypoints_Bushing(demo_case);

% Inverse Kinematics initial position
initPosition = [ 0 pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]

for i=1:length(xyzTargets(1,:))
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                 'SO3', rotMatTarget{i}, ...
                                 'initial', initPosition ); 
end           
% kin.getFK('endeffector', posTargets(2,:))
%% gravity direction
[gravityVec] = HEBI_Arm_gravity(gravitysetting);

%% holdeffect setting
stiffness = 10 * ones(1,kin.getNumDoF());

%% log data setting
poscmdlog = [];posfbklog = [];
Telog = [];Felog = [];deflectionlog = [];Tc_poslog = [];Fc_poslog = [];Tc_forcelog = [];Fc_forcelog = [];Tmlog = [];
ControlSwitchlog = []; Xelog = []; Velog = []; Xdlog = []; Vdlog = [];T_gripperlog = [];smoothing_factorlog = [];Fmlog = [];

%% trajectory & control
%%%%%%%%%%%%%%%%%%%%% go from here to initial waypoint %%%%%%%%%%%%%%%%%%%%
%control setting
fbk = group.getNextFeedback(); %�ʱ��ڼ� ����

waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(1)] );
t0 = fbk.time;
t = 0;
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedbackFull();
    fbk_gripper = grippergroup.getNextFeedbackFull(); 

    t = fbk.time - t0;
    [pos,vel,acc] = trajectory.getState(t);
    
    %get Jacobian
    J = kin.getJacobian('endeffector',fbk.position);

    %get external torque and Wrench
    Te = fbk.deflection'.*[130 170 70 70 70 70]'; %Hebi spring constant
    Fe = inv(J')*Te;             
    
    % Account for external efforts due to the gas spring
    effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
    
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
                                                
    Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
    Tc = -gains.positionKp.*(fbk.position - pos);% -gains.velocityKp.*(fbk.velocity - vel);
    
    cmd.position = [];
    cmd.velocity = [];
    cmd.effort = Tm + Tc;    
    group.send(cmd);    
    
    % �׸��� ����        
    grippercmd.position = [];
    grippercmd.velocity = [];
    grippercmd.effort = gripperforce(1);    
    grippergroup.send(grippercmd);  
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% go next waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial condition
idlePos = fbk.position;
ControlSwitch = [0 0 0 0 0 0]; % �ʱⰪ, ��ġ����� ����
Xd = [xyzTargets(:,1);pi;0;0]'; %cartesian space control�� ���� xd�ʱⰪ
Xe = Xd; %�̺κ��� ������ FK���� ��ǥ���� ���� �� ���� ���Ƿ� �ִ� ��. �����δ� Xe�� Xd�� ��Ȯ�� �������� ���� ������ �������̴�
for i=1:size(xyzTargets,2)-1
    
    % ������� �۵����� ���� ���� target position�� �������� ������ ��� ���� ��ġ���� ���� ��ġ�� ����
    % ���� ���� while������ ControlSwitch�� 1�̾��ٸ� target position�� �������� ���������̹Ƿ�, ������ġ�� �������� waypoint ����
%     if abs(posTargets(i,2) - fbk.position(2)) > 0.1
    if any(ControlSwitch) %���� while������ ����� �۵��Ǿ��ٸ�
        waypoints = [ fbk.position ;
                      posTargets(i+1,:) ];
    else        
        waypoints = [ posTargets(i,:) ;
                      posTargets(i+1,:) ];
    end
    
    trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    t0 = fbk.time;
    t = 0;    
    ControlSwitch = [0 0 0 0 0 0]; % ���� target���� �̵��� ���� �׻� ��ġ����� ����  
    smoothing_factor = 1;
    while t < trajectory.getDuration
        
        % Get feedback and update the timer
        fbk = group.getNextFeedbackFull();
        fbk_gripper = grippergroup.getNextFeedbackFull(); 
        t = fbk.time - t0;
        [pos,vel,acc] = trajectory.getState(t);
        
        %get Jacobian
        J = kin.getJacobian('endeffector',fbk.position);
        
        %get endeffector position%velocity
        %������ joint position���κ��� cartesian position�� �ٷ� �˼������� ������ API�� ����.
        dt = 1/group.getFeedbackFrequency;
        Ve = (J * fbk.velocity')';
        Xe = Xe + Ve * dt;
        
        %get external torque exerted on joint and endeffector
        Te = fbk.deflection.*[130 170 70 70 70 70];
        Fe = (inv(J')*Te')';       
      
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
%         effortOffset = [0 -7 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);
                                                    
        %get endeffector desired position%velocity
        %�������� ��쿡�� ��ġ���� �ϴ� axis���� ��ġ���� Xd�� �ؾ� ��. ������ 1axis �̵��̴ϱ� ��� ������.
       if any(ControlSwitch) %�������� ���      
            Xd = Xd .* ([1 1 1 1 1 1] - ControlSwitch) + Xe .* ControlSwitch; % ������� ��ġ����� ����ġ �ɶ� Ƣ�� ���� ������, ������� Xe->Xd�� �������� �����Ƿ�, ���� ��ġ�� Xd�� �ٸ� ���� �����Ƿ� ���� �ʿ�
            Vd = Vd .* ([1 1 1 1 1 1] - ControlSwitch) + Ve .* ControlSwitch; % ������� ��ġ����� ����ġ �ɶ� Ƣ�� ���� ������
        else %��ġ������ ���
            Jd = kin.getJacobian('endeffector',pos); %pos�� feedback pos�ƴ�, desired pos
            Vd = (Jd * vel')';
            Xd = Xd + Vd * dt;
        end            
        
        %������ �۵� ����   
        if i == 5 %�̴� ��쿡�� ������ ��ü ���� ��������
            for i_switch = 1 : length(Fe) %6�࿡ ���� Ȯ��
                if ControlSwitch(i_switch) == 0 %���� ��ġ ������ ��쿡�� ������� ����
                    if desired_force(i_switch) > 0 %desired force�� 0���� ū ��� 
                        if Fe(i_switch) >= desired_force(i_switch)
                            ControlSwitch(i_switch) = 1
                            i_switch = i_switch
                        end
                    elseif desired_force(i_switch) < 0 %desired force�� 0���� ���� ��� 
                        if Fe(i_switch) <= desired_force(i_switch)
                            ControlSwitch(i_switch) = 1
                            i_switch = i_switch
                        end
                    end                        
                end
            end
        end
        
        if i == 5
            if trajectory.getDuration - t <= smoothing_duration
                smoothing_factor = 1/smoothing_duration * (trajectory.getDuration - t);
%                 smoothing_factor = 1;   
            end
        end            
        
%         ePgain = [300 300 450 20 20 10];
        ePgain = [300 300 400 20 20 10];
        eVgain = [0.1 0.1 0.1 0.1 0.1 0.1];        
        
        %���� �� �Է�
        Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
        Fm = (inv(J')*Tm')';       
        
        %��ġ����
        if controlmode == 1 %cartesian sapce control           
            Fc_pos = -ePgain .* (Xe - Xd) - eVgain .* (Ve - Vd); 
            Fc_pos = Fc_pos .* ([1 1 1 1 1 1] - ControlSwitch); %������ ���� ��ġ��� 0���� ����
            Tc_pos = (J' * Fc_pos')';
        else %joint space control            
            Tc_pos = -gains.positionKp.*(fbk.position - pos)  -gains.velocityKp.*(fbk.velocity - vel);
        end
        
        %������, ControlSwitch�� ���� �ุ ������ ����
%         Fc_force = ([0 0.5 0 0 0 0].*(Fe - desired_force)+[0 16 0 0 0 0]).*ControlSwitch * smoothing_factor; 
%         Tc_force = (J' * Fc_force')';
    
        %feedforward term ���, computed torque control strucure���� �ʿ��� Fext�� �ٷ� ���
%         Fc_force = ([0 2.0 0 0 0 0].*(Fe - desired_force)).*ControlSwitch * smoothing_factor; 
%         Tc_force = (J' * Fc_force')'- (J' * (Fe.*ControlSwitch* smoothing_factor)')';
        
        %feedforward term ���, computed torque control strucure���� �ʿ��� Fext�� �ٷ� ���
        Fc_force = (([0 2.0 0 0 0 0].*(Fe - desired_force.*[1 smoothing_factor 1 1 1 1])) - Fe).*ControlSwitch; 
        Tc_force = (J' * Fc_force')';

        
        cmd.position = [];
        cmd.velocity = [];
        cmd.effort = Tm + Tc_pos + Tc_force;        
        group.send(cmd);   
        
        % �׸��� ����        
        grippercmd.position = [];
        grippercmd.velocity = [];
        grippercmd.effort = gripperforce(i+1); 
        grippergroup.send(grippercmd); 
        T_gripper = fbk_gripper.deflection.*[130];

        % data log
        poscmdlog = [poscmdlog; pos];
        posfbklog = [posfbklog; fbk.position];    
        deflectionlog = [deflectionlog;fbk.deflection];
        Telog = [Telog;Te];
        Felog = [Felog;Fe];   
        Tmlog = [Tmlog;Tm];
        Tc_poslog = [Tc_poslog;Tc_pos];
        if controlmode == 1
            Fc_poslog = [Fc_poslog;Fc_pos];
        end
        Tc_forcelog = [Tc_forcelog;Tc_force];
        Fc_forcelog = [Fc_forcelog;Fc_force];
        Xelog = [Xelog;Xe];
        Velog = [Velog;Ve];
        Xdlog = [Xdlog;Xd];
        Vdlog = [Vdlog;Vd];       
        ControlSwitchlog = [ControlSwitchlog;ControlSwitch];    
        T_gripperlog = [T_gripperlog;T_gripper];
        smoothing_factorlog = [smoothing_factorlog;smoothing_factor];
        Fmlog = [Fmlog;Fm];
    end    
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();

% HebiUtils.plotLogs( log, 'position');
% HebiUtils.plotLogs( log, 'velocity');
% HebiUtils.plotLogs( log, 'effort');

% subplot6a([0:dt:(size(Telog,1)-1)*dt],Telog,'Te',control_time)    
subplot6a([0:dt:(size(Felog,1)-1)*dt],Felog,'Fe',control_time)  

% % % % subplot6a([0:dt:(size(Tmlog,1)-1)*dt],Tmlog,'Tm',control_time)
% % % 
% % % subplot6a([0:dt:(size(Tc_poslog,1)-1)*dt],Tc_poslog,'Tc pos',control_time) 
% % % subplot6a([0:dt:(size(Fc_poslog,1)-1)*dt],Fc_poslog,'Fc pos',control_time) 
% % % subplot6a([0:dt:(size(Tc_forcelog,1)-1)*dt],Tc_forcelog,'Tc force',control_time) 
% % % subplot6a([0:dt:(size(Fc_forcelog,1)-1)*dt],Fc_forcelog,'Fc force',control_time) 
% % % subplot6a([0:dt:(size(Tmlog,1)-1)*dt],Tmlog + Tc_poslog + Tc_forcelog,'Tc',control_time) 
% % % 
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],posfbklog-poscmdlog,'poserror',control_time)
% % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],Xelog-Xdlog,'Xerror',control_time)
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],Velog-Vdlog,'Verror',control_time)
% % % 
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],posfbklog,'posfbk',control_time)
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],poscmdlog,'poscmd',control_time)
% % % 
% % % % subplot6a([0:dt:(size(Xelog,1)-1)*dt],Xelog,'Xe',control_time) 
% % % % subplot6a([0:dt:(size(Velog,1)-1)*dt],Velog,'Ve',control_time) 
% % % % subplot6a([0:dt:(size(Xdlog,1)-1)*dt],Xdlog,'Xd',control_time) 
% % % % subplot6a([0:dt:(size(Vdlog,1)-1)*dt],Vdlog,'Vd',control_time) 
% % % 
% % % % subplot6a([0:dt:(size(deflectionlog,1)-1)*dt],deflectionlog,'defelction',control_time) 
% % % 
% % % % figure;plot([0:dt:(size(T_gripperlog,1)-1)*dt],-T_gripperlog) 
% % % % figure;plot([0:dt:(size(ControlSwitchlog,1)-1)*dt],ControlSwitchlog(:,2)) 
% % % % figure;plot([0:dt:(size(smoothing_factorlog,1)-1)*dt],smoothing_factorlog) 
% % % 
% % % % save("IROS_short.mat",'poscmdlog',"posfbklog","Telog","Felog","deflectionlog","Tc_poslog","Fc_poslog","Tc_forcelog","Fc_forcelog","Tmlog","ControlSwitchlog","Xelog","Velog","Xdlog","Vdlog",'T_gripperlog','smoothing_factorlog','xyzTargets','rotMatTarget','control_time','gripperforce','Fmlog')
% % % % load forcecontrol.mat


