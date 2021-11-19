clc
clear

%%%01-31-2021. MZ have investigated all the signs and variabes
addpath './fun'


% rosinit
ros_vars = ros_setup();


% callbacks = ros_setup();

hydro = hydrodynamic_terms();

%%5 dof
%%x, y, z, roll(no motion), pitch yaw

%%Global variables won't change
Alpha.state.u=0;
Alpha.state.v=0;
Alpha.state.w=0;
Alpha.state.udot=0;
Alpha.state.vdot=0;
Alpha.state.wdot=0;
Alpha.state.q=0;
Alpha.state.r=0;
Alpha.state.qdot=0;
Alpha.state.rdot=0;
Alpha.state.roll=0;
Alpha.state.pitch=0;
Alpha.state.yaw=0;
Alpha.state.pitch_dot=0;
Alpha.state.yaw_dot=0;
Alpha.power = 0;
Earth.x=0;
Earth.y=0;
Earth.z=0;
Earth.xdot=0;
Earth.ydot=0;
Earth.zdot=0;

%%Alpha dimension
Alpha.dimension.L = 55*25.4/1000; %%length
Alpha.dimension.r = 5*25.4/1000;  %%radius

dt = 0.02;
Alpha.state.dt=dt;

%%initial values
%%computing inertial
%%Mass components
mass.hull.m = 16;
mass.s_b.m = 3; %3kg stationary battery
mass.m_b.m = 6; %%movable battery mass

mass.hull.x=0;
mass.hull.y=0;
mass.hull.z=0.5*25.4/1000; %%tune to lower CG

mass.s_b.x = -6 * 25.4/1000;
mass.s_b.z = -2.25 * 25.4/1000; %%inch to m
mass.s_b.y = 0;                 %%inch to m

%%%%%%%%%%%%%%%%Control values%%%%%%%%%%%%%%%%%%%%%%%%%
mass.m_b.x = 3*25.4/1000;  %%%moveable battery position [3.5 to -6.5]
mass.m_b.vx = 0;  %%%m/s moving speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mass.m_b.y = 0;
mass.m_b.z = 2.25*25.4/1000;

mass.total = mass.hull.m+mass.m_b.m+mass.s_b.m;

%%%restoring force
%%%%%%%%%%%%%%%%Control values%%%%%%%%%%%%%%%%%%%
mass.v_pump = 0;
mass.W_pump = 0;
mass.W_pump = mass.W_pump + mass.v_pump *dt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

B = mass.total *9.81 + mass.W_pump*9.81;
W = mass.total *9.81;

%%%%%%%%%%%%%%%%%Control values%%%%%%%%%%%%%%%%%%%
%%%thruster stuff

Thruster.Xpwm=1500;
Thruster.Ypwm=1500;
Thruster.Zpwm=1500;
Thruster.XT=PWM2T(Thruster.Xpwm); %%stern
Thruster.YT=PWM2T(Thruster.Ypwm); %%back horizontal
Thruster.ZT=PWM2T(Thruster.Zpwm); %front vertical

Alpha.control.state=0; %%regular AUV mode; 1-vertical mode; 2-transition.
Alpha.control.u_e_i=0;  %%integration error in u.
Alpha.control.pitch_e_i=0;  %%integration error in u.
Alpha.control.yaw_e_i=0;  %%integration error in u.
Alpha.control.z_e_i=0;  %%integration error in u.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vdata=[]; %%Vehicle data
Edata=[]; %%earth data
Cdata=[]; %%control data

% for t=0:dt:inf
t = 0;
while true
    t = t + dt;
    tic


    %%compute buoyancy and weight
    B = mass.total *9.81 + mass.W_pump*9.81;
    W = mass.total *9.81;

    %%inertial
    MI.Ixx = 1/2*mass.hull.m * Alpha.dimension.r.^2 ...
             + mass.s_b.m * (mass.s_b.y.^2+mass.s_b.z^2);
             + mass.m_b.m * (mass.m_b.y.^2+mass.m_b.z^2);

    MI.Iyy = 1/12*mass.hull.m * Alpha.dimension.L.^2 ...
             + mass.s_b.m * (mass.s_b.x.^2+mass.s_b.z^2);
             + mass.m_b.m * (mass.m_b.x.^2+mass.m_b.z^2);

    MI.Izz = 1/12*mass.hull.m * Alpha.dimension.L.^2 ...
             + mass.s_b.m * (mass.s_b.x.^2+mass.s_b.y^2);
             + mass.m_b.m * (mass.m_b.x.^2+mass.m_b.y^2);

    MI.Ixy = - mass.s_b.m * (mass.s_b.x * mass.s_b.y);
             - mass.m_b.m * (mass.m_b.x * mass.m_b.y);

    MI.Ixz = - mass.s_b.m * (mass.s_b.x * mass.s_b.z);
             - mass.m_b.m * (mass.m_b.x * mass.m_b.z);

    MI.Iyz = - mass.s_b.m * (mass.s_b.y * mass.s_b.z);
             - mass.m_b.m * (mass.m_b.y * mass.m_b.z);
    %%cetner of mass
    MI.XG = (mass.s_b.m*mass.s_b.x + mass.m_b.m*mass.m_b.x ...
            + mass.hull.m*mass.hull.x)/mass.total;
    MI.YG = (mass.s_b.m*mass.s_b.y + mass.m_b.m*mass.m_b.y ...
            + mass.hull.m*mass.hull.y)/mass.total;
    MI.ZG = (mass.s_b.m*mass.s_b.z + mass.m_b.m*mass.m_b.z ...
            + mass.hull.m*mass.hull.z)/mass.total;

    %%Thruster induced torque   
    Thruster.MT= -Thruster.XT*MI.ZG - Thruster.ZT * (20*25.4/1000 - MI.XG);
    Thruster.NT=Thruster.YT * (-20*25.4/1000 + MI.XG); %%Tunnel thrust * (location - XG)

    %%%full- equation
    % Alpha.state.udot = 1/(W/9.81 + hydro.added_mass(1)) * ...
    %         (Thruster.XT - hydro.damping(1) * Alpha.state.u * abs(Alpha.state.u) ...
    %           -W/9.81 * (Alpha.state.q * Alpha.state.w - Alpha.state.r*Alpha.state.v ...
    %                     + Alpha.state.qdot * MI.ZG ...
    %                     - (Alpha.state.q^2 + Alpha.state.r.^2) * MI.XG) ... %%rigid body
    %                     +(-W+B)*sin(Alpha.state.pitch) );  %restoring force

    Alpha.state.udot = 1/(W/9.81 + hydro.added_mass(1)) * ...
            (Thruster.XT - hydro.damping(1) * Alpha.state.u * abs(Alpha.state.u) ...
              -W/9.81 * (0 * Alpha.state.w - Alpha.state.r*Alpha.state.v ...
                        + 0 * MI.ZG ...
                        - (0 + 0) * MI.XG) ... %%rigid body
                        +(-W+B)*sin(Alpha.state.pitch) );  %restoring force

    Alpha.state.vdot = 1/(W/9.81 + hydro.added_mass(2)) * ...
             (Thruster.YT - hydro.damping(2) * Alpha.state.v * abs(Alpha.state.v) ...
              -W/9.81 * (Alpha.state.r * Alpha.state.u ...
                        + Alpha.state.rdot * MI.XG + Alpha.state.r * MI.ZG * Alpha.state.q) );

    Alpha.state.wdot = 1/(W/9.81 + hydro.added_mass(3)) * ...
            (Thruster.ZT - hydro.damping(3) * Alpha.state.w * abs(Alpha.state.w) ...
             -W/9.81 * (-Alpha.state.q * Alpha.state.u - Alpha.state.qdot * MI.XG - Alpha.state.q^2*MI.ZG)...
             - (-W+B) * cos(Alpha.state.pitch) );



    Alpha.state.qdot = 1/(MI.Iyy + hydro.added_mass(5)) * ...
            (Thruster.MT - hydro.damping(5) * Alpha.state.q * abs(Alpha.state.q) ...
             -MI.Ixz * Alpha.state.r^2 ...
             - W/9.81 * ( MI.ZG * (Alpha.state.udot + Alpha.state.q*Alpha.state.w - Alpha.state.r * Alpha.state.v)...
                         -MI.XG * (Alpha.state.wdot - Alpha.state.q * Alpha.state.u) ) ...
              - MI.ZG*B * sin(Alpha.state.pitch) - MI.XG * B* cos(Alpha.state.pitch) );    
    % Alpha.state.qdot = 1/(MI.Iyy + hydro.added_mass(5)) * ...
    %         (Thruster.MT - hydro.damping(5) * Alpha.state.q * abs(Alpha.state.q) ...
    %          -MI.Ixz * Alpha.state.r *abs(Alpha.state.r) ...
    %          - W/9.81 * ( MI.ZG * (Alpha.state.udot + Alpha.state.q*Alpha.state.w - Alpha.state.r * Alpha.state.v)...
    %                      -MI.XG * (Alpha.state.wdot - Alpha.state.q * Alpha.state.u) ) ...
    %           - MI.ZG*B * sin(Alpha.state.pitch) - MI.XG * B* cos(Alpha.state.pitch) );

    Alpha.state.rdot = 1/(MI.Izz + hydro.added_mass(6) ) * ...
            (Thruster.NT - hydro.damping(6) * Alpha.state.r * abs(Alpha.state.r) ...
            + MI.Ixz * Alpha.state.q * Alpha.state.r ...
            - W/9.81 * MI.XG * (Alpha.state.vdot + Alpha.state.r * Alpha.state.u) );

    %%update velocities
    Alpha.state.u = Alpha.state.u + dt * Alpha.state.udot;
    Alpha.state.v = Alpha.state.v + dt * Alpha.state.vdot;
    Alpha.state.w = Alpha.state.w + dt * Alpha.state.wdot;
    Alpha.state.q = Alpha.state.q + dt * Alpha.state.qdot;
    Alpha.state.r = Alpha.state.r + dt * Alpha.state.rdot;

    %%kinematics to get euler angle
    Alpha.state.pitch_dot = Alpha.state.q;
    Alpha.state.yaw_dot = cos(Alpha.state.pitch)*Alpha.state.r;
    Alpha.state.pitch = Alpha.state.pitch + dt * Alpha.state.pitch_dot;
    Alpha.state.yaw = Alpha.state.yaw + dt * Alpha.state.yaw_dot;
    Alpha.state.pitch=wrapTo2Pi(Alpha.state.pitch);
    Alpha.state.yaw=wrapTo2Pi(Alpha.state.yaw);


    %%Kinematics to get position in earth
    Rbe = rotz(Alpha.state.yaw*180/pi) * roty(Alpha.state.pitch*180/pi);
    Earth.xdot=Rbe(1,:)*[Alpha.state.u; Alpha.state.v; Alpha.state.w];
    Earth.ydot=Rbe(2,:)*[Alpha.state.u; Alpha.state.v; Alpha.state.w];
    Earth.zdot=Rbe(3,:)*[Alpha.state.u; Alpha.state.v; Alpha.state.w];

    Earth.x = Earth.x + Earth.xdot *dt;
    Earth.y = Earth.y + Earth.ydot *dt;
    Earth.z = Earth.z + Earth.zdot *dt;

    %%log  data
    Vdata = [Vdata; t, Alpha.state.u, Alpha.state.v, Alpha.state.w, ...  %%1-4
                   Alpha.state.udot, Alpha.state.vdot, Alpha.state.wdot, ...%%5-7
                   Alpha.state.q, Alpha.state.r, ...                        %%8-9
                   Alpha.state.qdot, Alpha.state.rdot, ...                  %%10-11
                   Alpha.state.pitch, Alpha.state.yaw, Alpha.power];                     %%12-14
    Edata =[Edata; t, Earth.xdot, Earth.ydot, Earth.zdot, ...
                      Earth.x,    Earth.y,    Earth.z];
    Cdata = [Cdata; ...
             t, Thruster.Xpwm, Thruster.Ypwm, Thruster.Zpwm,...
                mass.v_pump, mass.W_pump,...
                mass.m_b.vx, mass.m_b.x];

    

    %%%%%%%%%%%%%%%%%%%buoyancy change%%%%%%%%%%%%%%%%%%%%%%%%%
    %mass.v_pump = -0.01;
    mass.W_pump = mass.W_pump + mass.v_pump * dt;
    if(mass.W_pump>0.3)
        mass.W_pump=0.3;
    elseif(mass.W_pump<-0.3)
        mass.W_pump = -0.3;
    end
    %%%%%%%%movable battery position change%%%%%%%%%%%%%%%%%%%%
    %mass.m_b.vx = -0.01;  %%%m/s moving speed
    mass.m_b.x = mass.m_b.x + mass.m_b.vx * dt;
    if(mass.m_b.x >3.25*25.4/1000)
        mass.m_b.x = 3.25*25.4/1000;
    elseif (mass.m_b.x < -6.5*25.4/1000)
        mass.m_b.x = -6.5*25.4/1000;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%% ROS system %%%%%%%%%%%%%%%%%%%%%%%%%
    
    try
        ros_vars.thrust_msg = receive(ros_vars.thrust_sub, 0.0001);
        Thruster.Xpwm = ros_vars.thrust_msg.X;
        Thruster.Ypwm = ros_vars.thrust_msg.Y;
        Thruster.Zpwm = ros_vars.thrust_msg.Z;
    catch
        % todo: implement timeout based on last message arrival time
    end

    try
        ros_vars.shutdown_msg = receive(ros_vars.shutdown_sub, 0.0001);
    catch
        % do nothing
    end
    
    try
        ros_vars.odom_msg.Header.FrameId = 'odom';
        ros_vars.odom_msg.ChildFrameId = 'base_link';
        ros_vars.odom_msg.Header.Stamp = ros_vars.node.CurrentTime;

        ros_vars.odom_msg.Pose.Pose.Position.X = Earth.x;
        ros_vars.odom_msg.Pose.Pose.Position.Y = Earth.y;
        ros_vars.odom_msg.Pose.Pose.Position.Z = Earth.z;
        
        ros_vars.odom_msg.Twist.Twist.Linear.X = Alpha.state.u;
        ros_vars.odom_msg.Twist.Twist.Linear.Y = Alpha.state.v;
        ros_vars.odom_msg.Twist.Twist.Linear.Z = Alpha.state.w;
        
        ros_vars.odom_msg.Twist.Twist.Angular.Y = Alpha.state.qdot;
        ros_vars.odom_msg.Twist.Twist.Angular.Z = Alpha.state.rdot;
        
        quat = eul2quat([Alpha.state.yaw, Alpha.state.pitch, Alpha.state.roll]);
        
        ros_vars.odom_msg.Pose.Pose.Orientation.W = quat(1);
        ros_vars.odom_msg.Pose.Pose.Orientation.X = quat(2);
        ros_vars.odom_msg.Pose.Pose.Orientation.Y = quat(3);
        ros_vars.odom_msg.Pose.Pose.Orientation.Z = quat(4);
        
        
        send(ros_vars.odom_pub, ros_vars.odom_msg);
        
        ros_vars.tf_msg.Header= ros_vars.odom_msg.Header;
        ros_vars.tf_msg.ChildFrameId = ros_vars.odom_msg.ChildFrameId;
        
        ros_vars.tf_msg.Transform.Translation.X = ...
            ros_vars.odom_msg.Pose.Pose.Position.X;
        ros_vars.tf_msg.Transform.Translation.Y = ...
            ros_vars.odom_msg.Pose.Pose.Position.Y;
        ros_vars.tf_msg.Transform.Translation.Z = ...
            ros_vars.odom_msg.Pose.Pose.Position.Z;
     
        
        ros_vars.tf_msg.Transform.Rotation.W = ...
            ros_vars.odom_msg.Pose.Pose.Orientation.W;
        ros_vars.tf_msg.Transform.Rotation.X = ...
            ros_vars.odom_msg.Pose.Pose.Orientation.X;
        ros_vars.tf_msg.Transform.Rotation.Y = ...
            ros_vars.odom_msg.Pose.Pose.Orientation.Y;
        ros_vars.tf_msg.Transform.Rotation.Z = ...
            ros_vars.odom_msg.Pose.Pose.Orientation.Z;
        
        
        sendTransform(ros_vars.tftree, ros_vars.tf_msg);
    catch e
        fprintf(1,'The identifier was:\n%s',e.identifier);
        fprintf(1,'There was an error! The message was:\n%s',e.message);
    end
    
    if ros_vars.shutdown_msg.Data
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%Add guidance command here%%%%%%%%%%%%%%%%%%%%%%%%%
    % [Alpha, mass.m_b.vx, mass.v_pump,Thruster] = control_command(Alpha, Earth, mass, t, dt ,Thruster);
    [Thruster.XT,Thruster.Xpwm] = PWM2T(Thruster.Xpwm);
    [Thruster.ZT,Thruster.Zpwm] = PWM2T(Thruster.Zpwm);
    [Thruster.YT,Thruster.Ypwm] = PWM2T(Thruster.Ypwm);
    Alpha.power = PWM2POW(Thruster.Xpwm) + PWM2POW(Thruster.Ypwm) + PWM2POW(Thruster.Zpwm);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Thruster.XT
    while toc < dt
    end
    toc
end

    