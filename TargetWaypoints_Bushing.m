function [xyzTargets, rotMatTarget, control_time, gripperforce] = TargetWaypoints(demo_case)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 부분 작업 모듈화 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 지그로부터 부싱 pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    pick.xyzTargets =   [ 0.300   0.300   0.300   0.270;    % x [m]
                         -0.245  -0.245  -0.245  -0.245;    % y [m]
                         -0.080  -0.187  -0.187   0.000];  % z [m]  
    pick.gripperforce = [-5       2      -5      -5];  %동작 시작시에 해당 힘 완료함    
    pick.control_time = [ 3       3.0     0.8     1]; 
    pick.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi) R_x(pi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    %%%%%%%%%%%%%%%%%%%%%%%%% 그리퍼를 투입구 센터 기준으로 부싱 투입 %%%%%%%%%%%%%%%%%%%%
                                   %1       2       3      4       
    insert_center.xyzTargets =    [ 0.470   0.470   0.470  0.470;    % x [m]
                                    0.066   0.066   0.066  0.066;    % y [m]
                                    0.100  -0.020  -0.020  0.070];  % z [m]   
    insert_center.gripperforce =  [-5      -5       3      3    ];  %동작 시작시에 해당 힘 완료함 
    insert_center.control_time =  [ 1.5     1       0.8    1    ]; 
    rot_i = R_x(pi)*R_y(pi/5);
    insert_center.rotMatTarget = [ R_x(pi)  rot_i   rot_i  rot_i];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     %%%%%%%%%%%%%%%%%%%% 그리퍼를 투입구 한쪽 끝 기준으로 부싱 투입%%%%%%%%%%%%%%%%%%%%%%%
%                                    %1       2       3      4       
%     insert_end.xyzTargets =    [ 0.470   0.470   0.470  0.470;    % x [m]
%                                  0.088   0.088   0.088  0.088;    % y [m]
%                                  0.100  -0.020  -0.020  0.070];  % z [m]   
%     insert_end.gripperforce =  [-5      -5       3      3    ];  %동작 시작시에 해당 힘 완료함 
%     insert_end.control_time =  [ 1.5     1.5     1.5    1    ]; 
%     rot_i = R_x(pi)*R_y(pi/5);
% %     insert_end.rotMatTarget = [ R_x(pi)  rot_i   rot_i  rot_i];
%     insert_end.rotMatTarget = [ R_x(pi) R_x(pi) rot_i rot_i];
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%% 그리퍼를 투입구 한쪽 끝 기준으로 부싱 투입%%%%%%%%%%%%%%%%%%%%%%%
                                   %1       2       3      4       
    insert_end.xyzTargets =    [ 0.470   0.470   0.470  0.470;    % x [m]
                                 0.088   0.104   0.104  0.104;    % y [m]
                                 0.100  -0.020  -0.020  0.070];  % z [m]   
    insert_end.gripperforce =  [-5      -5       3      3    ];  %동작 시작시에 해당 힘 완료함 
    insert_end.control_time =  [ 1.5     1.5     0.5    1    ]; 
    rot_i = R_x(pi-pi/8);
%     insert_end.rotMatTarget = [ R_x(pi)  rot_i   rot_i  rot_i];
    insert_end.rotMatTarget = [ R_x(pi)    rot_i   rot_i  rot_i];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 부싱 pushing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             %1       2       3       4  
%     pushing.xyzTargets =   [ 0.530   0.530   0.530   0.530;    % x [m]
%                             -0.100   0.000   0.100  -0.100;    % y [m]
%                             -0.035  -0.035  -0.035  -0.035];  % z [m]
%     pushing.gripperforce = [-5      -5      -5      -5 ];           
%     pushing.control_time = [ 1.5     1.5     2.0     2.5 ];
%     pushing.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi) R_x(pi)];
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 부싱 pushing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            %1       2       3       4  
    pushing.xyzTargets =   [ 0.530   0.530   0.530;    % x [m]
                            -0.100   0.200  -0.050;    % y [m]
                            -0.035  -0.035  -0.035];  % z [m]
    pushing.gripperforce = [-5      -1.5    -5 ];           
    pushing.control_time = [ 1.5     3.0     1.0 ];
    pushing.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 부싱 pivoting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                             %1       2       3          
    pivoting.xyzTargets =   [ 0.530   0.530   0.530;    % x [m]
                             -0.100  -0.100  -0.100;    % y [m]
                             -0.030  -0.030  -0.030];  % z [m]
    pivoting.gripperforce = [-3      -0.3    -5    ];           
    pivoting.control_time = [ 1.0     2.0     1.5  ];
    pivoting.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 바닥에 부싱 place %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                          %1       2          
    place.xyzTargets =   [ 0.500   0.500;    % x [m]
                          -0.110  -0.110;    % y [m]
                          -0.150  -0.020 ];  % z [m]
    place.gripperforce = [-5       2     ];           
    place.control_time = [ 2.5     1.5   ];
    place.rotMatTarget = [ R_x(pi) R_x(pi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    if demo_case == 1
            %%%%%%%%%%%%%%%%%%%%%% 특수메탈 데모 pick&insert %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %            1       2       3       4       5       6       7      8            
        xyzTargets =   [ 0.300   0.300   0.300   0.270   0.470   0.470   0.470  0.470;    % x [m]
                        -0.245  -0.245  -0.245  -0.245   0.066   0.066   0.066  0.066;    % y [m]
                        -0.080  -0.190  -0.190   0.000   0.100  -0.020  -0.020  0.070];  % z [m]  
        gripperforce = [-5       2      -5      -5      -5       -5      3      3    ];  %동작 시작시에 해당 힘 완료함      
        control_time = [ 3       3.0     0.8     1       1.5      1      0.8    1    ]; 
        
        for i = 1 : size(xyzTargets,2)
            if i==6 || i==7 || i==8
                rotMatTarget{i} = R_x(pi)*R_y(pi/5); %#ok<*AGROW>
            else
                rotMatTarget{i} = R_x(pi);
            end
        end

    elseif demo_case == 2
            %%%%%%%%%%%%%%%% 특수메탈 데모 pick&pushing&pivoting&place %%%%%%%%%%%%%%%%%%
            %            1       2       3       4       5       6       7       8       9       10      11      12      13          
        xyzTargets =   [ 0.300   0.300   0.300   0.270   0.530   0.530   0.530   0.530   0.530   0.530   0.500   0.500;    % x [m]
                        -0.245  -0.245  -0.245  -0.245  -0.100   0.000   0.050  -0.100  -0.100  -0.100  -0.110  -0.110;    % y [m]
                        -0.080  -0.190  -0.190   0.000  -0.030  -0.030  -0.030  -0.030  -0.030  -0.030  -0.150  -0.020 ];  % z [m]
        gripperforce = [-5       2      -5      -5      -5      -5      -1      -3      -0.3    -5      -5       2     ];           
        control_time = [ 3       3       0.8     1       1.5     1.5     2.5     2.5     2.0     1.5     2.5     1.5   ];

        for i = 1 : size(xyzTargets,2)
            if i==0 
                rotMatTarget{i} = R_x(pi)*R_y(pi/5);
            else
                rotMatTarget{i} = R_x(pi);
            end
        end
        
    elseif demo_case == 11
            %%%%%%%%%%%%%%%%%%% 특수메탈 데모 pick&insert 모듈화 버전 %%%%%%%%%%%%%%%
        xyzTargets =       [pick.xyzTargets   insert_center.xyzTargets];
        gripperforce =     [pick.gripperforce insert_center.gripperforce];           
        control_time =     [pick.control_time insert_center.control_time];        
        rotMatTarget_Set = [pick.rotMatTarget insert_center.rotMatTarget]; 
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end    
        
    elseif demo_case == 12
            %%%%%%%% 특수메탈 데모 pick&pushing&pivoting&place 모듈화 버전 %%%%%%%%%%
        xyzTargets =       [pick.xyzTargets     pushing.xyzTargets    pivoting.xyzTargets    place.xyzTargets];
        gripperforce =     [pick.gripperforce   pushing.gripperforce  pivoting.gripperforce  place.gripperforce];           
        control_time =     [pick.control_time   pushing.control_time  pivoting.control_time  place.control_time];        
        rotMatTarget_Set = [pick.rotMatTarget   pushing.rotMatTarget  pivoting.rotMatTarget  place.rotMatTarget]; 
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end     
        
    elseif demo_case == 13
            %%%%%%%% 특수메탈 데모 pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick.xyzTargets     pushing.xyzTargets    insert_end.xyzTargets  ];
        gripperforce =     [pick.gripperforce   pushing.gripperforce  insert_end.gripperforce];           
        control_time =     [pick.control_time   pushing.control_time  insert_end.control_time];        
        rotMatTarget_Set = [pick.rotMatTarget   pushing.rotMatTarget  insert_end.rotMatTarget]; 
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end          
        
    elseif demo_case == 99
            %%%%%%%% 특수메탈 데모 pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pushing.xyzTargets(:,end)   insert_end.xyzTargets  ];
        gripperforce =     [pushing.gripperforce(:,end) insert_end.gripperforce];           
        control_time =     [pushing.control_time(:,end) insert_end.control_time];        
        rotMatTarget_Set = [pushing.rotMatTarget(:,end-2:end) insert_end.rotMatTarget]; 
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end    
    end
end

