// 서버의 prefix와 회수 시계열을 점화 기준 T 시간축으로 연결한다.
export function assembleMission(data) {
  const m = data.mission;
  const rows = data.trajectory.map((r, i) => {
    const ended = i === data.trajectory.length - 1 && !data.summary.landed;
    const ground = r.altitude_m === 0 && data.summary.landed;
    return {
      ...r,
      t_s: m.recovery_start_s + r.t_s,
      recovery_t_s: r.t_s,
      phase: ground ? "GROUND" : "GUIDED_DESCENT",
      phase_label: ground ? "착지·회수" : "회수 비행",
      recovery_active: !ground,
      canopy_fraction: 1,
      vertical_velocity_mps: -r.sink_mps,
      thrust_n: 0,
      data_source: "RECOVERY_MODEL",
      flight_model_valid: true,
      ...(ground
        ? { guidance_valid: false, guidance_status: "TOUCHDOWN_SNAPSHOT" }
        : {}),
      ...(ended
        ? {
            phase: "TIME_LIMIT",
            phase_label: "미착지·시간 종료",
            recovery_active: false,
            guidance_valid: false,
            guidance_status: "SIMULATION_ENDED",
          }
        : {}),
    };
  });
  if (data.summary.landed && m.end_s > rows.at(-1).t_s)
    rows.push({
      ...rows.at(-1),
      t_s: m.end_s,
      vertical_velocity_mps: 0,
      sink_mps: 0,
      airspeed_mps: 0,
      data_source: "ILLUSTRATIVE_GROUND_HOLD",
      flight_model_valid: false,
    });
  return [...m.prefix, ...rows];
}
