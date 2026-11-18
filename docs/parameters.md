# 파라미터 목록

전체 기본값은 [`simulation/config.json`](../simulation/config.json), 허용 범위는 [`configuration.validate()`](../simulation/configuration.py)에 있다. 예제 JSON에는 바꿀 항목만 적어도 된다.

측정·파일 근거와 가정을 구분하는 설명은 [물리 환경](physics.md)과 [기술 참고](reference.md)를 함께 읽는다. 단위가 없는 잡음·응답 설정도 실측 사양으로 간주하지 않는다.

| 키 | 기본값 | 의미·단위 |
| --- | --- | --- |
| `initialization` | `ork` | 상승: ork / measured / theoretical |
| `mode` | `target` | 제어 모드: target / manual / neutral / heading |
| `dt_s` | `0.02` | 적분 간격 (s) |
| `duration_s` | `180` | 최대 계산 시간 (s) |
| `stabilization_height_loss_m` | `20` | 전개·안정화 고도 손실 (m) |
| `initial_heading_deg` | `0` | 시작 방위 (°) |
| `wind_east_mps` | `1.5` | 동쪽 바람 (m/s) |
| `wind_north_mps` | `0.5` | 북쪽 바람 (m/s) |
| `left_command_deg` | `0` | 수동 왼쪽 회전각 (°) |
| `right_command_deg` | `180` | 수동 오른쪽 회전각 (°) |
| `command_start_s` | `2` | 수동 당김 시작 (s) |
| `command_end_s` | `12` | 수동 당김 종료 (s) |
| `target_heading_deg` | `90` | 목표 방위 (°) |
| `controller_period_s` | `0.05` | 제어 주기 (s) |
| `heading_kp` | `1.2` | 방위 비례 이득 |
| `yaw_kd_s` | `0.5` | 선회 감쇠 이득 (s) |
| `drum_radius_m` | `0.01` | 드럼 반지름 (m) |
| `pulley_ratio` | `1` | 도르래 비율 |
| `max_angle_deg` | `270` | 기구 권취 제한 (°) |
| `no_load_speed_deg_s` | `270` | 무부하 속도 (°/s) |
| `stall_torque_nm` | `2.941995` | 정지 토크 (N·m) |
| `servo_supply_voltage_v` | `6.5` | 공급 전압 · 설계값 (V) |
| `motor_performance_voltage_v` | `12` | 토크·속도 자료 전압 (V) |
| `efficiency` | `0.8` | 기구 효율 |
| `line_slack_m` | `0.002` | 줄 유격 (m) |
| `line_stiffness_n_m` | `2000` | 줄 강성 (N/m) |
| `control_load_fraction` | `0.15` | 조향줄 하중 비율 |
| `brake_stiffness_n_m` | `100` | 당김 하중 기울기 (N/m) |
| `canopy_area_m2` | `1.8` | 유효 면적 (m²) |
| `cl` | `0.8` | 양력 계수 CL |
| `cd` | `0.3` | 항력 계수 CD |
| `brake_cl_per_m` | `1` | 당김별 CL 증가 (1/m) |
| `brake_cd_per_m` | `2` | 당김별 CD 증가 (1/m) |
| `turn_gain_rad_s_m` | `4` | 선회 이득 (rad/s/m) |
| `turn_time_constant_s` | `0.8` | 선회 응답 시정수 (s) |
| `max_turn_rate_deg_s` | `35` | 선회율 한계 (°/s) |
| `max_bank_deg` | `40` | 뱅크각 한계 (°) |
| `max_valid_brake_m` | `0.08` | 보정 유효 당김 한계 (m) |
| `rocket_cd` | `0.6` | 근사 상승 로켓 CD |
| `rho_kg_m3` | `1.225` | 공기 밀도 (kg/m³) |
| `target_east_m` | `250` | 목표 동쪽 (m) |
| `target_north_m` | `350` | 목표 북쪽 (m) |
| `target_radius_m` | `25` | 목표 반경 (m) |
| `navigation_kp_s_inv` | `0.7` | Kp · 방위 오차 (1/s) |
| `navigation_kd` | `0.5` | Kd · 선회율 감쇠 |
| `controller_turn_gain_rad_s_m` | `4` | 제어기 보정 이득 (rad/s/m) |
| `control_brake_limit_m` | `0.04` | PD 최대 당김 (m) |
| `brake_slew_m_s` | `0.06` | 당김 변화 한계 (m/s) |
| `navigation_airspeed_mps` | `6.3` | 가정 수평 대기속도 (m/s) |
| `wind_estimator_tau_s` | `5` | 바람 추정 시정수 (s) |
| `wind_compensation` | `1` | 추정 바람 보상 (0~1) |
| `random_seed` | `7` | 바람·센서 난수 시드 |
| `gust_std_mps` | `0.6` | 돌풍 표준편차 (m/s) |
| `gust_time_constant_s` | `3` | 돌풍 상관 시간 (s) |
| `wind_shear_east_mps_100m` | `0.4` | 동쪽 바람 변화 (m/s/100m) |
| `wind_shear_north_mps_100m` | `0` | 북쪽 바람 변화 (m/s/100m) |
| `gps_period_s` | `0.2` | GPS 갱신 주기 (s) |
| `gps_latency_s` | `0.2` | GPS 지연 (s) |
| `gps_position_std_m` | `1.5` | GPS 위치 오차 σ (m) |
| `gps_velocity_std_mps` | `0.15` | GPS 속도 오차 σ (m/s) |
| `imu_period_s` | `0.02` | 방위·자이로 주기 (s) |
| `imu_latency_s` | `0.02` | 방위·자이로 지연 (s) |
| `heading_std_deg` | `0.8` | 방위 오차 σ (°) |
| `heading_bias_deg` | `0.3` | 방위 편향 (°) |
| `gyro_std_deg_s` | `0.2` | 선회율 오차 σ (°/s) |
| `baro_period_s` | `0.1` | 기압 고도 주기 (s) |
| `baro_latency_s` | `0.05` | 기압 고도 지연 (s) |
| `baro_std_m` | `0.5` | 고도 오차 σ (m) |
| `sensor_timeout_s` | `1.5` | 센서 시간초과 (s) |
| `gps_dropout_start_s` | `0` | GPS 끊김 시작 (s) |
| `gps_dropout_end_s` | `0` | GPS 끊김 끝 (s) |
| `mission_countdown_s` | `3` | 발사 전 대기 (s) |
| `deployment_transition_s` | `3` | 전개·안정화 표시 시간 (s) |
| `landing_hold_s` | `2` | 착지 결과 유지 (s) |
| `servo_latency_s` | `0.04` | 명령 지연 · 가정 (s) |
| `servo_accel_deg_s2` | `900` | 가감속 · 가정 (°/s²) |
| `servo_resolution_deg` | `0.087890625` | 명령 분해능 (°) |
| `wind_inertia_enabled` | `1` | 바람 변화 관성항 (0/1) |
| `gps_bias_std_m` | `0.8` | GPS 장기 편향 σ (m) |
| `gps_bias_time_constant_s` | `30` | GPS 편향 상관 시간 (s) |
| `gyro_bias_deg_s` | `0.1` | 자이로 편향 (°/s) |
| `rigging` | `unknown` | 줄 연결 기록: unknown / two_line / brake_lines |
| `body_heading_lag_s` | `0.25` | 회수 방위 지연 · 가정 (s) |
