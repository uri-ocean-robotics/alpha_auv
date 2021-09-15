function [v,F]=depth_control(v,earth)
    %%by moving battery
    %%we should use Claus's depth controller which controls pitch and depth at
    %%the same time.
    P=2;
    I=0.1;%0.1;
    D=5;%8;

    e = v.control.z - earth.z;

    % %%%remove e ambiguity
    % if(e>pi)
    % e = e-2*pi;
    % end
    % 
    % if(e<-pi)
    % e = e+2*pi;
    % end 

    v.control.z_e_i = v.control.z_e_i+e*v.state.dt;

    F = P * e + D * (-earth.zdot) + I * v.control.z_e_i;
    % F= -F;

end