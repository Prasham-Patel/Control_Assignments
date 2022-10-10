patel_progassignment1_matlab
eq_th_ddot1 = subs(sol.theta_ddot1, {l1 l2 r1 r2 m1 m2 I1 I2 g theta_dot1 theta_dot2 T1 T2}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81, 0, 0, 0, 0}); 
eq_th_ddot2 = subs(sol.theta_ddot2, {l1 l2 r1 r2 m1 m2 I1 I2 g theta_dot1 theta_dot2 T1 T2}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81, 0, 0, 0, 0}); 
eqi_th = ones(4, 2);
i = 1
while i < 5
    flag = 0;
    sol_th= vpasolve([eq_th_ddot1==0, eq_th_ddot2==0], [theta1, theta2], [-1 3.5; -1 3.5], "Random", true );
    for n = 1:i
        if ([round(sol_th.theta1, 4), round(sol_th.theta2, 4)] == [eqi_th(n, 1), eqi_th(n, 2)])
            flag = 1;
        end
    end
    if flag == 0
    eqi_th(i, 1) = round(sol_th.theta1, 4);
    eqi_th(i, 2) = round(sol_th.theta2, 4);
    i = i+1
    end
end
