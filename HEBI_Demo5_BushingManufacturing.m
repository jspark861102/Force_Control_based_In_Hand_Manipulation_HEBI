%힘제어 동작에 문제 없는지 확인
%피보팅은 어떻게 구현?

% 부싱 가공 데모 파일
% 힘제어에서 다시 위치제어로 돌아갈때 레퍼런스 문제를 해결함
% Z축의 경우 가스 스프링 때문에 위치 정확도 및 힘제어 구현이 좀 부자연스러운 면이 있음(Z축의 경우 중력과 가스 스프링의 평형으로 움직이지 않고 서있는 경우 많음)
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
desired_force = [1000 -13 1000 1000 1000 1000]; %힘제어 적용 하고 싶지 않은 축은 값을 크게 설정하자

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
fbk = group.getNextFeedback(); %초기자세 읽음

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
    
    % 그리퍼 제어        
    grippercmd.position = [];
    grippercmd.velocity = [];
    grippercmd.effort = gripperforce(1);    
    grippergroup.send(grippercmd);  
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% go next waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial condition
idlePos = fbk.position;
ControlSwitch = [0 0 0 0 0 0]; % 초기값, 위치제어로 시작
Xd = [xyzTargets(:,1);pi;0;0]'; %cartesian space control을 위한 xd초기값
Xe = Xd; %이부분이 문제다 FK에서 좌표값이 구할 수 없어 임의로 넣는 값. 실제로는 Xe가 Xd를 정확히 추종하지 못해 오차가 있을것이다
for i=1:size(xyzTargets,2)-1
    
    % 힘제어로 작동으로 인해 이전 target position에 도달하지 못했을 경우 현재 위치에서 다음 위치로 생성
    % 만약 이전 while문에서 ControlSwitch가 1이었다면 target position에 도달하지 못했을거이므로, 현재위치를 기준으로 waypoint 생성
%     if abs(posTargets(i,2) - fbk.position(2)) > 0.1
    if any(ControlSwitch) %이전 while문에서 힘제어가 작동되었다면
        waypoints = [ fbk.position ;
                      posTargets(i+1,:) ];
    else        
        waypoints = [ posTargets(i,:) ;
                      posTargets(i+1,:) ];
    end
    
    trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    t0 = fbk.time;
    t = 0;    
    ControlSwitch = [0 0 0 0 0 0]; % 다음 target으로 이동할 때는 항상 위치제어로 시작  
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
        %원래는 joint position으로부터 cartesian position을 바로 알수있으면 좋은데 API가 없다.
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
        %힘제어의 경우에도 위치제어 하는 axis들은 위치제어 Xd로 해야 됨. 지금은 1axis 이동이니까 상관 없지만.
       if any(ControlSwitch) %힘제어의 경우      
            Xd = Xd .* ([1 1 1 1 1 1] - ControlSwitch) + Xe .* ControlSwitch; % 힘제어에서 위치제어로 스위치 될때 튀는 현상 방지용, 힘제어는 Xe->Xd로 제어하지 않으므로, 현재 위치와 Xd가 다른 값을 가지므로 보상 필요
            Vd = Vd .* ([1 1 1 1 1 1] - ControlSwitch) + Ve .* ControlSwitch; % 힘제어에서 위치제어로 스위치 될때 튀는 현상 방지용
        else %위치제어의 경우
            Jd = kin.getJacobian('endeffector',pos); %pos는 feedback pos아님, desired pos
            Vd = (Jd * vel')';
            Xd = Xd + Vd * dt;
        end            
        
        %힘제어 작동 여부   
        if i == 5 %미는 경우에만 힘제어 교체 여부 결정하자
            for i_switch = 1 : length(Fe) %6축에 대해 확인
                if ControlSwitch(i_switch) == 0 %현재 위치 제어인 경우에만 힘제어로 변경
                    if desired_force(i_switch) > 0 %desired force가 0보다 큰 경우 
                        if Fe(i_switch) >= desired_force(i_switch)
                            ControlSwitch(i_switch) = 1
                            i_switch = i_switch
                        end
                    elseif desired_force(i_switch) < 0 %desired force가 0보다 작은 경우 
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
        
        %비선형 텀 입력
        Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
        Fm = (inv(J')*Tm')';       
        
        %위치제어
        if controlmode == 1 %cartesian sapce control           
            Fc_pos = -ePgain .* (Xe - Xd) - eVgain .* (Ve - Vd); 
            Fc_pos = Fc_pos .* ([1 1 1 1 1 1] - ControlSwitch); %힘제어 축은 위치제어값 0으로 세팅
            Tc_pos = (J' * Fc_pos')';
        else %joint space control            
            Tc_pos = -gains.positionKp.*(fbk.position - pos)  -gains.velocityKp.*(fbk.velocity - vel);
        end
        
        %힘제어, ControlSwitch가 켜진 축만 힘제어 수행
%         Fc_force = ([0 0.5 0 0 0 0].*(Fe - desired_force)+[0 16 0 0 0 0]).*ControlSwitch * smoothing_factor; 
%         Tc_force = (J' * Fc_force')';
    
        %feedforward term 대신, computed torque control strucure에서 필요한 Fext를 바로 사용
%         Fc_force = ([0 2.0 0 0 0 0].*(Fe - desired_force)).*ControlSwitch * smoothing_factor; 
%         Tc_force = (J' * Fc_force')'- (J' * (Fe.*ControlSwitch* smoothing_factor)')';
        
        %feedforward term 대신, computed torque control strucure에서 필요한 Fext를 바로 사용
        Fc_force = (([0 2.0 0 0 0 0].*(Fe - desired_force.*[1 smoothing_factor 1 1 1 1])) - Fe).*ControlSwitch; 
        Tc_force = (J' * Fc_force')';

        
        cmd.position = [];
        cmd.velocity = [];
        cmd.effort = Tm + Tc_pos + Tc_force;        
        group.send(cmd);   
        
        % 그리퍼 제어        
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


