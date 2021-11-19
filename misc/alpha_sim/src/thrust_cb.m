function thrust_cb(src, msg)
    global ThrustX;
    global ThrustY;
    global ThrustZ;
    
    msg = recieve('/sim/thruster_signal', 1);
    ThrustX = msg.pwm_x;
    ThrustY = msg.pwm_y;
    ThrustZ = msg.pwm_z;
end