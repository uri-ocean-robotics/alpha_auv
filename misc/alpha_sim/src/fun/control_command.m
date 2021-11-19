function [v, mbv, bpv, thrust] = control_command(v, earth, m, t, dt, thrust)

% v->alpha vehicle state
% mbv->movable battery velocity
% bpv->buoyancy pump velocity
% thrust->thruster vector

%%movable battery
%% 150 rpm, 8mm lead screw, 2mm pitch
%%movable battery
%% 100 rpm, 8mm lead screw, 2mm pitch
%% 1 inch housing radius

mbv=0;
bpv=0;

%%%case 1
%%% diving to depth
% if(t<=100)
%     mbv = -0.005; %%move battery backward
%     bpv = -0.01; %% retract buoyancy pump
%     thrust.XT=0;
%     thrust.YT=0;
%     thrust.ZT=0;
% end

% if(t>0)
%     mbv = 0.005;
%     if(m.W_pump>0)
%         bpv = -0.006;
%     elseif(m.W_pump<0)
%         bpv = 0.006;
%     end
%     v.control.pitch = 0/180*pi;
%     v.control.u = 0.5;
%     v.control.z = 60;
%     v.control.yaw = 30/180*pi;
%     [v, thrust.ZT]=depth_control(v,earth);
%     [v, thrust.XT]=surge_control(v);
%     [v, thrust.YT]=yaw_control(v);
%     
% end


%%test surge control
if(t>0)
    v.control.u = 0.5;
    [v, thrust.Xpwm] = surge_control(v);
    v.control.pitch = 20/180*pi;
    [v, thrust.Zpwm] = pitch_control(v);
    v.control.yaw = 30/180*pi;
    if(t>150)
    v.control.yaw = 90/180*pi;
    end
    [v, thrust.Ypwm]=yaw_control(v);
end
