function [v,F] = yaw_control(v)
    %%by moving battery

    P=800;
    I=10;
    D=300;

    e = v.control.yaw - v.state.yaw;

    %%%remove e ambiguity
    if(e>pi)
    e = e-2*pi;
    end

    if(e<-pi)
    e = e+2*pi;
    end 

    v.control.yaw_e_i = v.control.yaw_e_i+e*v.state.dt;

    F = P * e + D * (-v.state.yaw_dot) + I * v.control.yaw_e_i;
    F= -F+1500;

end
