function [rpy] = t2rpy(T_0_to_6,option)
    R = T_0_to_6(1:3,1:3);
    
    %자료마다 notation이 너무 다르다.
    %특히, siciliano의 경우, 'zyx'에서 z축 회전을 roll이라고 표현하고 있는데, 나의 notation과는 다르다
    %중요한 것은 이 함수와 rpy2w 함수가 roll pitch yaw 순서가 같으면 혼동 될 일은 없다.
    %나의 notation은 fixed frame에서 (fixed frame이므로 RzRyRx에서 Rx가 먼저 회전되는 것으로 보면 된다)
    %먼저 회전 되는 쪽을 roll로 보고 있다. 즉, Rz(yaw)Ry(pitch)Rx(roll)
    %rpy(1): roll, rpy(2): pitch, rpy(3): yaw
    switch option
        case 'zyx'
            % peter corke Library를 가져온 것으로, siciliano 책의 값과 비교 필요 (같겠지)
            % old ZYX order (as per Paul book)
            if abs(abs(R(3,1)) - 1) < eps  % when |R31| == 1
                % singularity

                rpy(1) = 0;     % roll is zero
                if R(3,1) < 0
                    rpy(3) = -atan2(R(1,2), R(1,3));  % R-Y
                else
                    rpy(3) = atan2(-R(1,2), -R(1,3));  % R+Y
                end
                rpy(2) = -asin(R(3,1));
            else
                rpy(1) = atan2(R(3,2), R(3,3));  % R
                rpy(3) = atan2(R(2,1), R(1,1));  % Y

                rpy(2) = atan2(-R(3,1)*cos(rpy(1)), R(3,3));
                
                % siciliano book
                if abs(rpy(2)) > pi/2 %(pi/2, 3pi/2)
                    rpy(3) = atan2(-R(2,1), -R(1,1));  % Y
                    rpy(1) = atan2(-R(3,2), -R(3,3));  % R
                    rpy(2) = atan2(-R(3,1), -sqrt(R(3,2)^2 + R(3,3)^2));
                end

            end
            
        case 'xyz'
            % peter corke Library를 가져온 것으로, siciliano 책의 값과 비교 필요 (같겠지)
            % XYZ order
            if abs(abs(R(1,3)) - 1) < eps  % when |R13| == 1
                % singularity
                rpy(1) = 0;  % roll is zero
                if R(1,3) > 0
                rpy(3) = atan2( R(3,2), R(2,2));   % R+Y
                else
                     rpy(3) = -atan2( R(2,1), R(3,1));   % R-Y
                end
                rpy(2) = asin(R(1,3));
            else
                rpy(1) = -atan2(R(1,2), R(1,1));
                rpy(3) = -atan2(R(2,3), R(3,3));
                
                rpy(2) = atan2(R(1,3)*cos(rpy(1)), R(1,1));
            end
            
        case 'zyz'
%             if abs(R(1,3)) < eps && abs(R(2,3)) < eps
            if abs(abs(R(3,3)) - 1) < eps 
                % singularity
                rpy(1) = 0;
                sp = 0;
                cp = 1;
                rpy(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
                rpy(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
            else
                % non singular

                % Only positive phi is returned.
                rpy(1) = atan2(R(2,3), R(1,3));
%                 rpy(1) = atan2(-R(2,3), -R(1,3));

                sp = sin(rpy(1));
                cp = cos(rpy(1));
                rpy(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
                rpy(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
                
                % siciliano book
                if rpy(2) > pi || rpy(2) < 0
                    rpy(1) = atan2(-R(2,3), -R(1,3));
                    rpy(2) = atan2(-sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
                    rpy(3) = atan2(-R(3,2),R(3,1));
                end
            end
    end
    
    rpy = rpy';

end
