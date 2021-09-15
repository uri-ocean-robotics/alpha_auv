function P = PWM2POW (x)

    %%cftool for BR thruster. x->pwm. T->thrust [N]
    %%same as PWM2T
    pwm_l=1300;  %%min = 1100
    pwm_r=1700;  %%max = 1900

    if(x>pwm_r)
        x=pwm_r;
    elseif(x<pwm_l)
        x=pwm_l;
    end
           p1 =   1.752e-10 ;
           p2 =  -1.055e-06  ;
           p3 =    0.002465  ;
           p4 =      -2.641  ;
           p5 =        1088  ;
    P = p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5;
    P = P+0.9377;%%move the power upward with the minimum value
    if(x>1465 & x<1535)
        P=0;
    end
    P=P*12;

end