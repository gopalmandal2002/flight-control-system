%% flight_pitch_roll.m
close all; clear; clc;

%% ----- PARAMETERS (example, change if you have real data) -----
% Pitch
a1 = -0.65; a2 = -1.2; bp = 2.0;
A_p = [0 1; a1 a2]; B_p = [0; bp]; C_p = [1 0]; D_p = 0;
sys_p = ss(A_p,B_p,C_p,D_p);
G_p = tf(sys_p); % theta(s)/delta_e(s)

% Roll
a3 = -0.8; a4 = -1.5; br = 1.5;
A_r = [0 1; a3 a4]; B_r = [0; br]; C_r = [1 0]; D_r = 0;
sys_r = ss(A_r,B_r,C_r,D_r);
G_r = tf(sys_r); % phi(s)/delta_a(s)

%% ----- OPEN-LOOP ANALYSIS -----
disp('--- Pitch Open-loop poles ---'); disp(eig(A_p));
disp('--- Roll Open-loop poles ---'); disp(eig(A_r));

figure; rlocus(G_p); title('Root Locus - Pitch');
figure; margin(G_p); title('Bode & Margins - Pitch'); [GMp,PMp,Wcg,Wcp] = margin(G_p);
fprintf('Pitch GM = %g dB, PM = %g deg\n',20*log10(GMp),PMp);

figure; rlocus(G_r); title('Root Locus - Roll');
figure; margin(G_r); title('Bode & Margins - Roll'); [GMr,PMr,~,~] = margin(G_r);
fprintf('Roll GM = %g dB, PM = %g deg\n',20*log10(GMr),PMr);

%% ----- SIMPLE CLASSICAL DESIGN EXAMPLE (Pitch) -----
% PD controller (example)
Kp = 30; Kd = 2;
s = tf('s');
C_pd = Kp + Kd*s;
figure; rlocus(C_pd*G_p); title('Root Locus - Pitch with PD');
K_choice = 40;
sys_cl_pd = feedback(K_choice*C_pd*G_p,1);
figure; step(sys_cl_pd); title('Closed-loop Step - Pitch (PD)'); stepinfo(sys_cl_pd)

%% ----- STATE-FEEDBACK and LQR (Pitch) -----
% Check controllability
rank_ctrb = rank(ctrb(A_p,B_p));
fprintf('Pitch controllability rank = %d (max 2)\n', rank_ctrb);

% Pole placement
des_poles = [-2+1j, -2-1j];
K_place = place(A_p,B_p,des_poles);
sys_cl_place = ss(A_p - B_p*K_place, B_p, C_p, 0);
figure; step(sys_cl_place); title('State-feedback (pole place) - Pitch'); stepinfo(sys_cl_place)

% LQR
Q = diag([100, 10]); R = 1;
K_lqr = lqr(A_p,B_p,Q,R);
sys_cl_lqr = ss(A_p - B_p*K_lqr, B_p, C_p, 0);
figure; step(sys_cl_lqr); title('LQR - Pitch'); stepinfo(sys_cl_lqr)

%% ----- COMPARE RESPONSES (overlay) -----
t = 0:0.01:10;
[y_pd, t] = step(sys_cl_pd, t);
[y_place, ~] = step(sys_cl_place, t);
[y_lqr, ~] = step(sys_cl_lqr, t);

figure; plot(t,y_pd,'LineWidth',1.5); hold on;
plot(t,y_place,'--','LineWidth',1.3);
plot(t,y_lqr,':','LineWidth',1.3); grid on;
legend('PD (K=40)','State-place','LQR'); xlabel('Time (s)'); ylabel('Pitch (rad)');
title('Controller Comparison - Pitch');

%% ----- ROBUSTNESS: param sweep (Pitch) -----
% Vary a1,a2 ±20% and plot closed-loop poles for LQR
a1s = a1*(0.8:0.05:1.2);
a2s = a2*(0.8:0.05:1.2);
figure; hold on; title('Closed-loop poles (LQR) under param variation - Pitch');
for aa1 = a1s
    for aa2 = a2s
        Ap = [0 1; aa1 aa2];
        sys_tmp = ss(Ap - B_p*K_lqr, B_p, C_p, 0);
        poles_tmp = eig(Ap - B_p*K_lqr);
        plot(real(poles_tmp), imag(poles_tmp),'.');
    end
end
xlabel('Real'); ylabel('Imag'); grid on;

disp('Script finished.');
