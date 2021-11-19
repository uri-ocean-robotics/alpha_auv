function [T,x] = PWM2T (x)

    %%cftool for BR thruster. x->pwm. T->thrust [N]
    pwm_l=1300;  %%min = 1100
    pwm_r=1700;  %%max = 1900

    if(x>pwm_r)
        x=pwm_r;
    elseif(x<pwm_l)
        x=pwm_l;
    end
    p1 =   -4.08e-12  ;
    p2 =   5.154e-08  ;
    p3 =  -0.0001736 ;
    p4 =      0.2326  ;
    p5 =      -111.6 ;
    T = (p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5)*9.81;

    if(x>1465 & x<1535)
        T=0;
    end
end