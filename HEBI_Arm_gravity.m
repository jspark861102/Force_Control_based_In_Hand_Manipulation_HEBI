function [gravityVec] = HEBI_Arm_gravity(gravitysetting)
    if gravitysetting == 1
            gravityVec = [0 0 -1];
    elseif gravitysetting == 2
        fbk = group.getNextFeedbackFull();
        baseRotMat = HebiUtils.quat2rotMat( [ 
            fbk.orientationW(1), ...
            fbk.orientationX(1), ...
            fbk.orientationY(1), ...
            fbk.orientationZ(1) ] );
        gravityVec = -baseRotMat(3,1:3);  
    end
end