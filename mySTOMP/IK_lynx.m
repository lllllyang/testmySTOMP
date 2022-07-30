% % Fill in this function with the mathematics for computing the positions of
% % each of the joints of the Lynx robot and of the gripper.
% %
% % input: T = 4x4 transformation matrix representing the pose of the end
% %             effector frame
% % output: q = 1x6 vector of joint variable values that bring the end
% %             effector frame to pose T
% %         is_possible = true/false whether it is possible for the end
% %             effector to move to the pose described by T
% %
% % Solution provided by: Nick Kneier (MEAM 520 2017 TA)
% 
% function [q, is_possible] = IK_lynx(T)
% 
% L1 = 3*25.4;          %base height (in in)
% L2 = 5.75*25.4;       %shoulder to elbow length (in in)
% L3 = 7.37*25.4;      %elbow to wrist length (in in)
% L4 = 1.7*25.4;       %Wrist1 to Wrist2 (in in)
% L5 = 1.25*25.4;       %wrist2 to base of gripper (in in)
% L6 = 1.125*25.4;      %gripper length
% D6 = L4 + L5 + L6;    %Distance from Wrist Center to Gripper Center
% PI = pi();            %Create PI constant
% 
% %Calculates the wrist center location
% xw = T(1,4) - D6*T(1,3);
% yw = T(2,4) - D6*T(2,3);
% zw = T(3,4) - D6*T(3,3);
% 
% %Calculates the theta 1 needed
% th1 = atan2(yw,xw);
% 
% %Parameters needed for calculating theta 2 and theta 3
% r = sqrt(xw^2 + yw^2);
% s = zw - L1;
% D = (r^2 + s^2 - L2^2 - L3^2)/(-2*L2*L3);
% % 
% % if D<=0
% %     disp('error')
% %     is_possible = false;
% %     q = [0 0 0 0 0 0];
% %     return
% % end
% 
% 
% %Calculates theta 3
% th3 = atan2(D,sqrt(1-D^2));
% 
% %Uses theta 3 to calculate theta 2
% th2 = atan2(r,s) - atan2(L3*cos(th3),L2-L3*sin(th3));
% 
% %Steps to get the rotation matrix from frame 3 to frame 6
% [~,A] = updateQ([th1,th2,th3,0,0,0]);
% T03 = A(:,:,1)*A(:,:,2)*A(:,:,3);
% Trans_R03 = T03(1:3,1:3)';
% R36 = Trans_R03*T(1:3,1:3);
% 
% %Calculates theta 4
% zdist = T(3,4) - zw;
% xydist = sqrt((T(1,4) - xw)^2 + (T(2,4) - yw)^2);
% th4 = -atan2(zdist, xydist) - th2 - th3;
% 
% %Calculates theta 5
% [~,B] = updateQ([th1,th2,th3,th4,0,0]);
% T04 = B(:,:,1)*B(:,:,2)*B(:,:,3)*B(:,:,4);
% Trans_R04 = T04(1:3,1:3)';
% R46 = Trans_R04*T(1:3,1:3);
% 
% th5  = atan2(R46(2,1),R46(1,1));
% 
% q = [th1, th2, th3, th4, th5, 0];
% 
% 
% %Finds the plane of the robot and calculates the vector normal to this
% %plane
% z_norm =[0;0;1];
% xy_norm = [xw;yw;0]/norm([xw;yw;0]);
% normal = cross(xy_norm,z_norm);
% 
% %Calculates to see if the plane's normal vector and the z6 axis are orthogonal
% %i.e. the the z6 axis  is entirely within the robot's plane
% dot_prod = round(dot(normal,T(1:3,3)),1);
% 
% %If the two vectors are orthoginal, the configuration is possible
% if dot_prod == 0
%     is_possible = true;
% %Otherwise, the configuration is not possible
% else
%     is_possible = false;
% end
% 
% end
function [q, is_possible] = IK_lynx(T)
    addpath('D:\from lab\2021-11-17\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
    temp=[0;48.2999158357778;0;-80.4847794064853;0;-42.7846952422630;0];
    init_theta1=180;
    init_theta2=0;
    xd=T(1,4);
    yd=T(2,4);
    zd=T(3,4);
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    
    which_need=abs(All_theta(7,:)) >= 140;
    need_plus=All_theta(7,which_need);
    if need_plus < -140
        All_theta(7,which_need)=need_plus+180;
    elseif need_plus > 140
        All_theta(7,which_need)=need_plus-180;
    end
    
    count_no=0;
            while isempty(All_theta)
                    if yd<0
                        init_theta2=init_theta2+1;
                        init_theta1=180-count_no;
                    elseif yd>0
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;  
                    end
                count_no=count_no+1;
                [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
            end
    [hang,lie]=size(All_theta);
    tott=1000;
    for each_lie =1:lie
        now=sum(abs(All_theta(:,each_lie)-temp));
        if now < tott
            tott=now;
            which=each_lie;
        end
    end
    to_add=All_theta(:,which);
    is_possible=true;
    q=to_add;
end