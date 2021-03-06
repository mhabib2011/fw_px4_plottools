function ControlPlots(sysvector)

%% My plots
figure()

plot3(sysvector('vehicle_local_position_0.x').Data, ...
    sysvector('vehicle_local_position_0.y').Data, ...
    -sysvector('vehicle_local_position_0.z').Data)
hold on;
plot3(sysvector('vehicle_local_position_setpoint_0.x').Data, ...
    sysvector('vehicle_local_position_setpoint_0.y').Data, ...
    -sysvector('vehicle_local_position_setpoint_0.z').Data)
legend('vehicle position', 'vehicle setpoint')
xlabel('x')
ylabel('y')
zlabel('z')



% ts_acceleration_controller
figure()
i=1;
ax1 = subplot(2,3,i);
hold on
plot(sysvector('vehicle_local_position_setpoint_0.x').Time, ...
    sysvector('vehicle_local_position_setpoint_0.x').Data);
plot(sysvector('vehicle_local_position_0.x').Time, ...
    sysvector('vehicle_local_position_0.x').Data);
hold off
legend ('setpoint x','position x')
i = i + 1;

ax2 = subplot(2,3,i);
hold on
plot(sysvector('vehicle_local_position_setpoint_0.y').Time, ...
    sysvector('vehicle_local_position_setpoint_0.y').Data);
plot(sysvector('vehicle_local_position_0.y').Time, ...
    sysvector('vehicle_local_position_0.y').Data);
hold off
legend ('setpoint y','position y')
i = i + 1;

ax3 = subplot(2,3,i);
hold on
plot(sysvector('vehicle_local_position_setpoint_0.z').Time, ...
    sysvector('vehicle_local_position_setpoint_0.z').Data);
plot(sysvector('vehicle_local_position_0.z').Time, ...
    sysvector('vehicle_local_position_0.z').Data);
hold off
legend ('setpoint z','position z')
i = i + 1;

ax4 = subplot(2,3,i);
hold on;
plot(sysvector('ts_controls_0.pos_err_0').Time, ...
    sysvector('ts_controls_0.pos_err_0').Data);
plot(sysvector('ts_controls_0.vel_cmd_0').Time, ...
    sysvector('ts_controls_0.vel_cmd_0').Data);
plot(sysvector('ts_controls_0.vel_err_0').Time, ...
    sysvector('ts_controls_0.vel_err_0').Data);
plot(sysvector('ts_controls_0.vel_int_0').Time, ...
    sysvector('ts_controls_0.vel_int_0').Data);
plot(sysvector('ts_controls_0.acc_cmd_0').Time, ...
    sysvector('ts_controls_0.acc_cmd_0').Data);
hold off;
legend('pos_err', 'vel cmd', 'vel err', 'vel int', 'acc_cmd')
i = i + 1;

ax5 = subplot(2,3,i);
hold on;
plot(sysvector('ts_controls_0.pos_err_1').Time, ...
    sysvector('ts_controls_0.pos_err_1').Data);
plot(sysvector('ts_controls_0.vel_cmd_1').Time, ...
    sysvector('ts_controls_0.vel_cmd_1').Data);
plot(sysvector('ts_controls_0.vel_err_1').Time, ...
    sysvector('ts_controls_0.vel_err_1').Data);
plot(sysvector('ts_controls_0.vel_int_1').Time, ...
    sysvector('ts_controls_0.vel_int_1').Data);
plot(sysvector('ts_controls_0.acc_cmd_1').Time, ...
    sysvector('ts_controls_0.acc_cmd_1').Data);
hold off;
legend('pos_err', 'vel cmd', 'vel err', 'vel int', 'acc_cmd')
i = i + 1;

ax6 = subplot(2,3,i);
hold on;
plot(sysvector('ts_controls_0.pos_err_2').Time, ...
    sysvector('ts_controls_0.pos_err_2').Data);
plot(sysvector('ts_controls_0.vel_cmd_2').Time, ...
    sysvector('ts_controls_0.vel_cmd_2').Data);
plot(sysvector('ts_controls_0.vel_err_2').Time, ...
    sysvector('ts_controls_0.vel_err_2').Data);
plot(sysvector('ts_controls_0.vel_int_2').Time, ...
    sysvector('ts_controls_0.vel_int_2').Data);
plot(sysvector('ts_controls_0.acc_cmd_2').Time, ...
    sysvector('ts_controls_0.acc_cmd_2').Data);
hold off;
legend('pos_err', 'vel cmd', 'vel err', 'vel int', 'acc_cmd')
linkaxes([ax1, ax2, ax3, ax4, ax5, ax6],'x')


% longitudinal and lateral error
figure()
i = 1;
ax1 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_local_position_setpoint_0.lat_err').Time, ...
    sysvector('vehicle_local_position_setpoint_0.lat_err').Data);
plot(sysvector('vehicle_local_position_setpoint_0.alt_err').Time, ...
    sysvector('vehicle_local_position_setpoint_0.alt_err').Data);
hold off;
legend('lat err', 'alt err')
i = i + 1; 

% attitude control
ax2 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.roll').Time, ...
    sysvector('vehicle_attitude_0.roll').Data*180/pi);
plot(sysvector('vehicle_attitude_setpoint_0.roll_body').Time, ...
    sysvector('vehicle_attitude_setpoint_0.roll_body').Data*180/pi);
hold off;
legend('roll Angle', 'roll Angle Ref')
i = i + 1; 

ax3 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.pitch').Time, ...
    sysvector('vehicle_attitude_0.pitch').Data*180/pi);
plot(sysvector('vehicle_attitude_setpoint_0.pitch_body').Time, ...
    sysvector('vehicle_attitude_setpoint_0.pitch_body').Data*180/pi);
hold off;
legend('pitch Angle', 'pitch Angle Ref')
i = i + 1;

ax4 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.yaw').Time, ...
    sysvector('vehicle_attitude_0.yaw').Data*180/pi);
plot(sysvector('vehicle_attitude_setpoint_0.yaw_body').Time, ...
    sysvector('vehicle_attitude_setpoint_0.yaw_body').Data*180/pi);
hold off;
legend('Yaw Angle', 'Yaw Angle Ref')
i = i + 1;

% Flight mode transit status
ax5 = subplot(3,4,i);
hold on;
plot(sysvector('vtol_vehicle_status_0.vtol_in_trans_mode').Time, ...
    sysvector('vtol_vehicle_status_0.vtol_in_trans_mode').Data);
plot(sysvector('vtol_vehicle_status_0.vtol_in_rw_mode').Time, ...
    sysvector('vtol_vehicle_status_0.vtol_in_rw_mode').Data);
plot(sysvector('vtol_vehicle_status_0.in_transition_to_fw').Time, ...
    sysvector('vtol_vehicle_status_0.in_transition_to_fw').Data);
legend('vtol in trans mode', 'vtol in rw mode', 'in transition to fw');
ylabel('Act. outputs []')
i = i + 1;

% attitude rate control
ax6 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.rollspeed').Time, ...
    rad2deg(sysvector('vehicle_attitude_0.rollspeed').Data));
plot(sysvector('vehicle_rates_setpoint_0.roll').Time, ...
    sysvector('vehicle_rates_setpoint_0.roll').Data);
hold off;
legend('p','pRef');
i = i + 1;

ax7 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.pitchspeed').Time, ...
    rad2deg(sysvector('vehicle_attitude_0.pitchspeed').Data));
plot(sysvector('vehicle_rates_setpoint_0.pitch').Time, ...
    sysvector('vehicle_rates_setpoint_0.pitch').Data);
hold off;
legend('q','qRef');
i = i + 1;

ax8 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_attitude_0.yawspeed').Time, ...
    rad2deg(sysvector('vehicle_attitude_0.yawspeed').Data));
plot(sysvector('vehicle_rates_setpoint_0.yaw').Time, ...
    sysvector('vehicle_rates_setpoint_0.yaw').Data);
hold off;
legend('r','rRef');
ylabel('Rates [deg/s]')
i = i + 1;

% actuator plots
ax9 = subplot(3,4,i);
hold on;
plot(sysvector('actuator_controls_0_0.control_0').Time, ...
    sysvector('actuator_controls_0_0.control_0').Data);
plot(sysvector('actuator_controls_0_0.control_1').Time, ...
    sysvector('actuator_controls_0_0.control_1').Data);
plot(sysvector('actuator_controls_0_0.control_2').Time, ...
    sysvector('actuator_controls_0_0.control_2').Data);
plot(sysvector('actuator_controls_0_0.control_3').Time, ...
    sysvector('actuator_controls_0_0.control_3').Data);
plot(sysvector('actuator_controls_0_0.control_4').Time, ...
    sysvector('actuator_controls_0_0.control_4').Data);
legend('u_{ail}','u_{elev}', 'u_{rud}', 'u_{throt}', 'u_{flaps}');
ylabel('Act. controls []')
i = i + 1;

ax10 = subplot(3,4,i);
hold on;
plot(sysvector('actuator_controls_1_0.control_0').Time, ...
    sysvector('actuator_controls_1_0.control_0').Data);
plot(sysvector('actuator_controls_1_0.control_1').Time, ...
    sysvector('actuator_controls_1_0.control_1').Data);
plot(sysvector('actuator_controls_1_0.control_2').Time, ...
    sysvector('actuator_controls_1_0.control_2').Data);
plot(sysvector('actuator_controls_1_0.control_3').Time, ...
    sysvector('actuator_controls_1_0.control_3').Data);
plot(sysvector('actuator_controls_1_0.control_4').Time, ...
    sysvector('actuator_controls_1_0.control_4').Data);
legend('u_{ail}','u_{elev}', 'u_{rud}', 'u_{throt}', 'u_{flaps}');
ylabel('Act. controls []')
i = i + 1;

ax11 = subplot(3,4,i);
hold on;
plot(sysvector('actuator_outputs_0.output_0').Time, ...
    sysvector('actuator_outputs_0.output_0').Data);
plot(sysvector('actuator_outputs_0.output_1').Time, ...
    sysvector('actuator_outputs_0.output_1').Data);
plot(sysvector('actuator_outputs_0.output_2').Time, ...
    sysvector('actuator_outputs_0.output_2').Data);
plot(sysvector('actuator_outputs_0.output_3').Time, ...
    sysvector('actuator_outputs_0.output_3').Data);
plot(sysvector('actuator_outputs_0.output_4').Time, ...
    sysvector('actuator_outputs_0.output_4').Data);
plot(sysvector('actuator_outputs_0.output_5').Time, ...
    sysvector('actuator_outputs_0.output_5').Data);
plot(sysvector('actuator_outputs_0.output_6').Time, ...
    sysvector('actuator_outputs_0.output_6').Data);
plot(sysvector('actuator_outputs_0.output_7').Time, ...
    sysvector('actuator_outputs_0.output_7').Data);
legend('motor 1','mototr 2', 'motor 3', 'motor 4', 'flaps', 'elevon 1', 'elevon 2', 'fake elevator');
ylabel('Act. outputs []')
i = i + 1;

ax12 = subplot(3,4,i);
hold on;
plot(sysvector('vehicle_status_flags_0.condition_global_position_valid'));
plot(sysvector('vehicle_status_flags_0.condition_home_position_valid'));
plot(sysvector('mission_result_0.valid'));
ylabel('Status Flags')
legend('global position valid','local position valid', 'mission result valid')
i = i + 1;

linkaxes([ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10, ax11, ax12],'x')


%% Display the low level controller data.
% 
% % cut rate and attitude setpoints
% pRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.roll'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% pRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
% qRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.pitch'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% qRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
% rRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.yaw'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% rRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
% 
% rollRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.roll_body'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% rollRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
% pitchRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.pitch_body'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% pitchRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
% yawRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.yaw_body'),...
%     sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
% yawRef.DataInfo.Interpolation = tsdata.interpolation('zoh');

% 
% %% %%%%%%%%%%%%%%%%%%%%%%
% % First Figure: Overall Control
% %%%%%%%%%%%%%%%%%%%%%%%%
% fig1 = figure();
% fig1.Name = 'Attitude+Airspd+Alt Control';
% nrSubplotSections = 12;
% plotmargins.horiz = 0.05;
% plotmargins.vert = 0.016;
% plotmargins.vert_last = 0.055;
% 
% % Mode and state plot
% axeshandle(1) = subplot_tight(nrSubplotSections,1,1,[plotmargins.vert plotmargins.horiz]);
% hold on;
% stairs(sysvector('commander_state_0.main_state').Time, sysvector('commander_state_0.main_state').Data);
% ylabel('Mode [-]')
% hold off
% 
% % Angle plots
% [pitch, roll, yaw, quatTime] = ...
%         QuaternionToEuler(sysvector('vehicle_attitude_0.q_0'), sysvector('vehicle_attitude_0.q_1'),...
%         sysvector('vehicle_attitude_0.q_2'), sysvector('vehicle_attitude_0.q_3'));
% 
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,2,[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(quatTime, rad2deg(yaw));
% plot(yawRef.Time, rad2deg(yawRef.Data));
% hold off;
% legend('Yaw Angle', 'Yaw Angle Ref')
% 
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[3 4],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(quatTime, rad2deg(roll));
% plot(rollRef.Time, rad2deg(rollRef.Data));
% plot(quatTime, rad2deg(pitch));
% plot(pitchRef.Time, rad2deg(pitchRef.Data));
% hold off;
% legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref');    
% ylabel('Attitude [deg]')
% 
% % rates plot
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[5 6],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(sysvector('vehicle_attitude_0.rollspeed').Time, rad2deg(sysvector('vehicle_attitude_0.rollspeed').Data));
% plot(pRef.Time, rad2deg(pRef.Data));
% plot(sysvector('vehicle_attitude_0.pitchspeed').Time, rad2deg(sysvector('vehicle_attitude_0.pitchspeed').Data));
% plot(qRef.Time, rad2deg(qRef.Data));
% plot(sysvector('vehicle_attitude_0.yawspeed').Time, rad2deg(sysvector('vehicle_attitude_0.yawspeed').Data));
% plot(rRef.Time, rad2deg(rRef.Data));
% hold off;
% legend('p','pRef','q','qRef','r','rRef');
% ylabel('Rates [deg/s]')
% 
% % actuator output plots
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[7 8],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(sysvector('actuator_controls_0.control_0').Time, sysvector('actuator_controls_0.control_0').Data);
% plot(sysvector('actuator_controls_0.control_1').Time, sysvector('actuator_controls_0.control_1').Data);
% plot(sysvector('actuator_controls_0.control_2').Time, sysvector('actuator_controls_0.control_2').Data);
% plot(sysvector('actuator_controls_0.control_3').Time, sysvector('actuator_controls_0.control_3').Data);
% plot(sysvector('actuator_controls_0.control_4').Time, sysvector('actuator_controls_0.control_4').Data);
% legend('u_{ail}','u_{elev}', 'u_{rud}', 'u_{throt}', 'u_{flaps}');
% ylabel('Act. outputs []')
% 
% % airspeed plots
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[9],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(sysvector('airspeed_0.true_airspeed_m_s').Time, sysvector('airspeed_0.true_airspeed_m_s').Data);
% plot(sysvector('airspeed_0.indicated_airspeed_m_s').Time, sysvector('airspeed_0.indicated_airspeed_m_s').Data);
% % plot(sysvector('tecs_status_0.airspeed_sp').Time, sysvector('tecs_status_0.airspeed_sp').Data);
% % TODO add here v ref nom, v ref min, v ref max
% legend('v_{TAS} [m/s]','v_{IAS} [m/s]', 'v_{TAS} ref[m/s]');
% ylabel('Airsp. [m/s]')
% 
% % altitude plots
% % axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[10 11],[plotmargins.vert plotmargins.horiz]);
% % plot(sysvector('vehicle_global_position_0.alt').Time, sysvector('vehicle_global_position_0.alt').Data);
% % hold on;
% % plot(sysvector('tecs_status_0.altitude_sp').Time, sysvector('tecs_status_0.altitude_sp').Data);
% % plot(sysvector('position_setpoint_triplet_0.current_alt').Time, sysvector('position_setpoint_triplet_0.current_alt').Data);
% % plot(sysvector('vehicle_gps_position_0.alt').Time, sysvector('vehicle_gps_position_0.alt').Data*fconv_gpsalt);
% % terrain_alt = sysvector('vehicle_global_position_0.terrain_alt').Data;
% % terrain_alt(sysvector('vehicle_global_position_0.terrain_alt_valid').Data == 0) = NaN;
% % plot(sysvector('vehicle_global_position_0.terrain_alt').Time, terrain_alt);
% % legend('Altitude estimate [m]', 'Alt. ref (smoothed)[m]', 'Alt. ref [m]','GPS Alt [m]','Terrain Altitude [m]');
% % xlabel('Time [s]')
% % ylabel('Alt. [m]')
% 
% % Plot configuration
% for i=1:length(axeshandle)-1; set(axeshandle(end-i),'XTickLabel',''); end;
% linkaxes(axeshandle(:),'x');
% set(axeshandle(:),'XGrid','on','YGrid','on','ZGrid','on');
% dcm_obj = datacursormode(fig1);
% set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%
% % Second Figure : TECS
% %%%%%%%%%%%%%%%%%%%%%%%%
% 
% fig1 = figure();
% fig1.Name = 'TECS status';
% nrSubplotSections = 14;
% plotmargins.horiz = 0.05;
% plotmargins.vert = 0.014;
% plotmargins.vert_last = 0.055;
% 
% % Mode and state plot
% axeshandle = zeros(0,0);
% axeshandle(1) = subplot_tight(nrSubplotSections,1,1,[plotmargins.vert plotmargins.horiz]);
% hold on;
% stairs(sysvector('commander_state_0.main_state').Time, sysvector('commander_state_0.main_state').Data);
% % stairs(sysvector('tecs_status_0.mode').Time, sysvector('tecs_status_0.mode').Data);
% ylabel('Mode [-]')
% legend('Autopilot mode','TECS mode');
% hold off
% 
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[2 3],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(quatTime, rad2deg(roll));
% plot(rollRef.Time, rad2deg(rollRef.Data));
% plot(quatTime, rad2deg(pitch));
% plot(pitchRef.Time, rad2deg(pitchRef.Data));
% % plot(sysvector('tecs_status_0.pitch_integ').Time, rad2deg(sysvector('tecs_status_0.pitch_integ').Data ./ (sysvector('tecs_status_0.airspeed_filtered').Data*5*9.81)));
% hold off;
% legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref', 'Pitch Ref I');    
% ylabel('Attitude [deg]')
% 
% % throttle output
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[4 5],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(sysvector('actuator_controls_0.control_3').Time, sysvector('actuator_controls_0.control_3').Data);
% % plot(sysvector('tecs_status_0.throttle_integ').Time, sysvector('tecs_status_0.throttle_integ').Data);
% plot(sysvector('actuator_controls_0.control_4').Time, sysvector('actuator_controls_0.control_4').Data);
% legend('u_{throt}', 'u_{throt,I}','u_{flaps}');
% ylabel('Act. outputs []')
% 
% % airspeed plots
% axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[6 7],[plotmargins.vert plotmargins.horiz]);
% hold on;
% plot(sysvector('airspeed_0.true_airspeed_m_s').Time, sysvector('airspeed_0.true_airspeed_m_s').Data);
% % plot(sysvector('tecs_status_0.airspeed_filtered').Time, sysvector('tecs_status_0.airspeed_filtered').Data);
% plot(sysvector('airspeed_0.indicated_airspeed_m_s').Time, sysvector('airspeed_0.indicated_airspeed_m_s').Data);
% % plot(sysvector('tecs_status_0.airspeed_sp').Time, sysvector('tecs_status_0.airspeed_sp').Data);
% % TODO add here v ref nom, v ref min, v ref max
% legend('v_{TAS} [m/s]','v_{TAS} (filtered) [m/s]','v_{IAS} [m/s]', 'v_{TAS} ref[m/s]');
% ylabel('Airsp.')
% 
% % % airspeed plots
% % axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[8 9],[plotmargins.vert plotmargins.horiz]);
% % hold on;
% % plot(sysvector('tecs_status_0.airspeed_derivative_sp').Time, sysvector('tecs_status_0.airspeed_derivative_sp').Data);
% % plot(sysvector('tecs_status_0.airspeed_derivative').Time, sysvector('tecs_status_0.airspeed_derivative').Data);
% % legend('dv/dt_{TAS,ref} [m/s²]','dv/dt_{TAS} [m/s²]');
% % ylabel('Airsp. der.')
% 
% % % Energy (and rate) errors
% % axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[10 11],[plotmargins.vert plotmargins.horiz]);
% % hold on;
% % plot(sysvector('tecs_status_0.total_energy_error').Time, sysvector('tecs_status_0.total_energy_error').Data);
% % plot(sysvector('tecs_status_0.total_energy_rate_error').Time, sysvector('tecs_status_0.total_energy_rate_error').Data);
% % plot(sysvector('tecs_status_0.energy_distribution_error').Time, sysvector('tecs_status_0.energy_distribution_error').Data);
% % plot(sysvector('tecs_status_0.energy_distribution_rate_error').Time, sysvector('tecs_status_0.energy_distribution_rate_error').Data);
% % legend('Total energy error','Total energy rate error','Energy distribution error','Energy distrib. rate error');
% % ylabel('Energy')
% 
% % % altitude plots
% % axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[12 13],[plotmargins.vert plotmargins.horiz]);
% % plot(sysvector('vehicle_global_position_0.alt').Time, sysvector('vehicle_global_position_0.alt').Data);
% % hold on;
% % plot(sysvector('tecs_status_0.altitude_sp').Time, sysvector('tecs_status_0.altitude_sp').Data);
% % plot(sysvector('position_setpoint_triplet_0.current_alt').Time, sysvector('position_setpoint_triplet_0.current_alt').Data);
% % terrain_alt = sysvector('vehicle_global_position_0.terrain_alt').Data;
% % terrain_alt(sysvector('vehicle_global_position_0.terrain_alt_valid').Data == 0) = NaN;
% % plot(sysvector('vehicle_global_position_0.terrain_alt').Time, terrain_alt);
% % legend('Altitude estimate [m]', 'Alt. ref (smoothed)[m]', 'Alt. ref [m]','Terrain Altitude [m]');
% % xlabel('Time [s]')
% % ylabel('Alt. [m]')
% 
% % Plot configuration
% for i=1:length(axeshandle)-1; set(axeshandle(end-i),'XTickLabel',''); end;
% linkaxes(axeshandle(:),'x');
% set(axeshandle(:),'XGrid','on','YGrid','on','ZGrid','on');
% dcm_obj = datacursormode(fig1);
% set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
% 
% end
% 
