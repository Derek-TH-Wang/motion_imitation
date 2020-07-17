function trans2xr3(dogtrot)

%     coxa = 0.032875;
%     femur = 0.25223;
%     tibia = 0.251;
%     laikagoJoint = dogtrot(:,8:length(dogtrot(1,:)));
%     laikago2KinematicsOffset=[-1,1,1, 1,1,1, -1,1,1, 1,1,1]; %3142
%     offsetAngle = [0, 0.6, -0.66];
%     for ii=1:length(dogtrot(:,1))
%         for jj = 1:4
%             angle = laikagoJoint(ii,(jj-1)*3+1:jj*3) + offsetAngle;
%             angle = angle.*laikago2KinematicsOffset(jj:jj+2);
%             if jj == 2 || jj == 4
%                 pos(ii,(jj-1)*3+1:jj*3) = FK(angle, coxa , femur , tibia, 1);
%             else
%                 pos(ii,(jj-1)*3+1:jj*3) = FK(angle, coxa , femur , tibia, -1);
%             end
%         end
%     end
%     plot3(pos(:,1),pos(:,2),pos(:,3))
%     hold on
%     plot3(pos(:,4),pos(:,5),pos(:,6))
%     plot3(pos(:,7),pos(:,8),pos(:,9))
%     plot3(pos(:,10),pos(:,11),pos(:,12))
%     axis equal
    

    inv =[0.0    1.0    0.0
      0.0    0.0    1.0
      1.0    0.0    0.0];
    pos = dogtrot(:,1:3);
    quat = dogtrot(:,4:7);
    rotm = quat2rotm(quat);
    for ii = 1:length(pos(:,1))
       res(:,:,ii) = inv * rotm(:, :, ii);
       quat1(ii, :) = rotm2quat(res(:, :, ii));
    end
    dogtrot1 = dogtrot;
    dogtrot1(:,4:6) = quat1(:,2:4);
    dogtrot1(:,7) = quat1(:,1);
    dogtrot1(:,[8,9,10,12,13,14,15,16,18,19]) = -dogtrot1(:,[8,9,10,12,13,14,15,16,18,19]);
    dogtrot1(:,[9,12,15,18])=dogtrot1(:,[9,12,15,18])-0.6;
    dogtrot1(:,[10,13,16,19])=dogtrot1(:,[10,13,16,19])+0.66;
    dogtrot1(:,3) = dogtrot1(:,3) - 0.13;
    joint = dogtrot1(:,8:length(dogtrot1(1,:)));
    
    
%     cnt = 0;
%     for ii=1:length(dogtrot1(:,1))
%         for jj = 1:4
%             cnt = cnt+1;
%             angle = joint(ii,(jj-1)*3+1:jj*3);
%             pos_l = FK(angle, coxa , femur , tibia, 1);
%             pos_r = FK(angle, coxa , femur , tibia, -1);
%             ang_l = IK(pos_l, coxa, femur, tibia, -1);
%             ang_r = IK(pos_r, coxa, femur, tibia, 1);
%             if max((ang_l-angle)) > 0.0001 || max((ang_r-angle)) > 0.0001
%                 print(angle)
%                 print((ang_l-angle))
%                 print((ang_r-angle))
%             end
%             disp(cnt)
%         end
%     end

    
    
    
    for ii = 1:length(pos(:,1))
        fprintf("  [")
        for jj = 1:19
            if jj == 19
                fprintf("%6.5f", dogtrot1(ii, jj));
            else
                fprintf("%6.5f, ", dogtrot1(ii, jj));
            end
        end
        if ii == length(pos(:,1))
            fprintf("]\n");
        else
            fprintf("],\n");
        end
    end
end

function D = checkdomain(D)
    if D > 1 || D < -1
        disp("____OUT OF DOMAIN____")
        if D > 1
            D = 0.99999999999999;
            return
        end
        if D < -1
            D = -0.99999999999999;
            return
        end
    else
        return
    end
end

function angles = IK(coord , coxa , femur , tibia, left_right)
    D = (coord(2)^2+(-coord(3))^2-coxa^2+(-coord(1))^2-femur^2-tibia^2)/(2*tibia*femur);
    D = checkdomain(D);
    gamma = atan2(-sqrt(1-D^2),D);
    tetta = -atan2(coord(3),coord(2))-atan2(sqrt(coord(2)^2+(-coord(3))^2-coxa^2),-left_right*coxa);
    if tetta > pi
        tetta = tetta - pi*2;
    end
    if tetta < -pi
        tetta = tetta + pi*2;
    end
    alpha = atan2(-coord(1),sqrt(coord(2)^2+(-coord(3))^2-coxa^2))-atan2(tibia*sin(gamma),femur+tibia*cos(gamma));
    angles = [-tetta, -alpha, -gamma];
end

function p = FK(angle, coxa , femur , tibia, left_right)
    sideSign = left_right;
    s1 = sin(angle(1));
    s2 = sin(angle(2));
    s3 = sin(angle(3));
    c1 = cos(angle(1));
    c2 = cos(angle(2));
    c3 = cos(angle(3));

    c23 = c2 * c3 - s2 * s3;
    s23 = s2 * c3 + c2 * s3;
    p0 = tibia * s23 + femur * s2;
    p1 = coxa * (sideSign) * c1 + tibia * (s1 * c23) + femur * c2 * s1;
    p2 = coxa * (sideSign) * s1 - tibia * (c1 * c23) - femur * c1 * c2;
    p = [p0, p1, p2];
end




