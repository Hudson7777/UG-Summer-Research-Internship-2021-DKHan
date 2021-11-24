

% create a open-loop baseline controller and view the step response
s = tf('s');
P = 1/(s^2 + 10*s + 20);
step(P)

% define a PID controller using transfer function
Kp = 350;
Ki = 300;
Kd = 50;
C = pid(Kp,Ki,Kd)
T = feedback(C*P,1);

t = 0:0.01:2;
step(T,t)

% tune the PID controller
pidTuner(P,C)
