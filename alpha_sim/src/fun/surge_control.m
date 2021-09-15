function [v,F]=surge_control(v)

    P=20;
    I=5;
    D=10;

    e = v.control.u - v.state.u;
    v.control.u_e_i = v.control.u_e_i+e*v.state.dt;

    F = P * e + D * (-v.state.udot) + I * v.control.u_e_i+1500;

end