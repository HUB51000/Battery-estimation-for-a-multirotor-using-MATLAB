alpha=20;
% alpha in degrees
mass=17;
% mass in kg
Cd=1.3;
Area=1.55;
% area is in m^2
g=9.8066;
% g in m/(s^2)
wind_velocity=5;
% wind velocity in m/s
area=1.55;
% area in m^2
T=mass*g/cosd(alpha)/6;
% Maximum achievable Thrust
T_sin_alpha_by_m=g*tand(alpha);
constant_for_drag=0.5*1.225*area*Cd;
K=constant_for_drag*sind(alpha)*(1/mass);
Drone_max_velocity=((mass*g/(cosd(alpha)*constant_for_drag))^0.5)-wind_velocity;

syms v
velocity_start=0;
diff_time_Acc=(1/(T_sin_alpha_by_m-(K*((wind_velocity+v)^2))));
% sind(alpha) term is for component of area
time_Acc(v) = int(diff_time_Acc);
final_time = int(diff_time_Acc,velocity_start,Drone_max_velocity);

velocity_time=finverse(time_Acc(v)-time_Acc(0));
drag_time = constant_for_drag*sind(alpha)*((velocity_time + wind_velocity)^2);
Acc_time = (g*tand(alpha)) - (drag_time/mass);

figure(1);
fplot(velocity_time,[0,double(final_time)]);
ylabel('Velocity(m/s)');
xlabel('Time(s)');
figure(2);
fplot(Acc_time,[0,double(final_time)]);
ylabel('Acceleration(m/s^2)');
xlabel('Time(s)');
figure(3);
fplot(drag_time,[0,double(final_time)]);
ylabel('Drag(N)');
xlabel('Time(s)');

