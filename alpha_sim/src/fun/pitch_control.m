function [v,F]=pitch_control(v)
    %%by moving battery

    P=20;
    I=5;%0.1;
    D=20;%8;

    e = v.control.pitch - v.state.pitch;

    %%%remove e ambiguity
    if(e>pi)
    e = e-2*pi;
    end

    if(e<-pi)
    e = e+2*pi;
    end 

    v.control.pitch_e_i = v.control.pitch_e_i+e*v.state.dt;

    F = P * e + D * (-v.state.pitch_dot) + I * v.control.pitch_e_i;
    F= -F+1500;

end

