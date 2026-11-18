import { drawTrack, drawMechanism, drawHistory } from "./ui/charts.js";
import { assembleMission } from "./ui/mission-timeline.js";
import { fields } from "./ui/config-fields.js";
("use strict");
let scene = null;
const $ = (id) => document.getElementById(id);

let result = null,
  configDefaults = null,
  playing = false,
  playTime = 0,
  lastFrame = null;
let missionTrajectory = [],
  neutralMissionTrajectory = [];

function setStatus(text, cls = "") {
  $("status").textContent = text;
  $("status").className = cls;
}
function readConfig() {
  const cfg = {};
  for (const [key, value] of new FormData($("controls")))
    cfg[key] = ["mode", "initialization", "rigging"].includes(key)
      ? value
      : Number(value);
  return cfg;
}
function fillConfig(cfg) {
  if ($("sensitivity-table").hasChildNodes())
    $("sensitivity-status").textContent =
      "설정 변경됨 · 이전 입력의 비교입니다.";
  if ($("sweep-table").hasChildNodes())
    $("sweep-status").textContent = "설정 변경됨 · 아래 표는 이전 계산입니다.";
  for (const [key, value] of Object.entries(cfg)) {
    const input = $("controls").elements.namedItem(key);
    if (input) input.value = value;
  }
}

function fullMission() {
  return $("view-scope").value === "mission";
}
function viewRows() {
  return fullMission() ? missionTrajectory : result.trajectory;
}
function neutralRows() {
  return fullMission() ? neutralMissionTrajectory : result.neutral.trajectory;
}
function currentIndex() {
  return Math.min(viewRows().length - 1, Number($("scrub").value));
}
function indexAt(t) {
  const rows = viewRows();
  let lo = 0,
    hi = rows.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (rows[mid].t_s <= t) lo = mid + 1;
    else hi = mid;
  }
  return Math.max(0, lo - 1);
}

function configureView() {
  if (!result) return;
  const rows = viewRows();
  playTime = rows[0].t_s;
  $("scrub").max = rows.length - 1;
  $("scrub").value = 0;
  lastFrame = null;
  scene?.setData({
    ...result,
    trajectory: rows,
    neutral: { ...result.neutral, trajectory: neutralRows() },
  });
  draw();
}
function seekMission(t) {
  if (!result) return;
  $("view-scope").value = "mission";
  configureView();
  const rows = viewRows();
  let index = rows.findIndex((r) => r.t_s >= t - 1e-8);
  if (index < 0) index = rows.length - 1;
  $("scrub").value = index;
  playTime = rows[index].t_s;
  playing = false;
  $("play").textContent = "▶ 재생";
  draw();
}
function missionNavigation() {
  const parent = $("mission-events");
  parent.replaceChildren();
  for (const e of result.mission.events) {
    const b = document.createElement("button");
    b.type = "button";
    b.dataset.phase = e.key;
    b.textContent = `${e.t_s < 0 ? "T − " : "T + "}${Math.abs(e.t_s).toFixed(1)}s · ${e.label}`;
    b.title = e.basis;
    b.addEventListener("click", () => seekMission(e.t_s));
    parent.append(b);
  }
}

function draw() {
  if (!result) return;
  const row = viewRows()[currentIndex()];
  $("clock").textContent = row.t_s.toFixed(1) + " s";
  drawTrack(row, result, viewRows(), neutralRows(), currentIndex());
  drawMechanism(row, result);
  drawHistory(row, result, viewRows(), fullMission());
  scene?.update(row);
  $("scene-time").textContent =
    (fullMission() ? "T " : "R ") +
    (row.t_s < 0 ? "− " : "+ ") +
    Math.abs(row.t_s).toFixed(1) +
    " s";
  $("scene-alt").textContent = "ALT " + row.altitude_m.toFixed(1) + " m";
  $("scene-attitude").textContent =
    `수직 대비 ${(row.body_tilt_deg ?? Math.abs(row.bank_deg)).toFixed(1)}° · 횡이동 ${Math.hypot(row.east_m, row.north_m).toFixed(1)} m`;
  $("scene-pull").textContent =
    `L ${(row.left_brake_m * 1000).toFixed(1)} mm · R ${(row.right_brake_m * 1000).toFixed(1)} mm`;
  const phase =
      row.phase ||
      (row.altitude_m === 0 && result.summary.landed
        ? "GROUND"
        : "GUIDED_DESCENT"),
    beforeRecovery = fullMission() && row.recovery_t_s < 0,
    ground = phase === "GROUND";
  $("scene-phase").textContent =
    row.phase_label || (phase === "GROUND" ? "착지·회수" : "회수 비행");
  $("mission-phase").textContent = row.phase_label || "회수 구간 단독 재생";
  $("mission-detail").textContent =
    phase === "TIME_LIMIT"
      ? "설정된 계산 시간이 끝났습니다. 아직 공중에 있으며 착지로 판정하지 않습니다."
      : phase === "PAD"
        ? "발사대 대기 · 준비 완료를 가정한 시나리오입니다. 실제 ARM·점화 명령은 전송하지 않습니다."
        : phase === "BOOST"
          ? "추력에 의한 상승 · 낙하산 접힘 · 조향 비활성"
          : phase === "COAST"
            ? "추력 종료 후 관성 상승 · 최고점까지 이동"
            : phase === "DEPLOY" || phase === "STABILIZATION"
              ? "가정한 전개·안정화 전환 · 팽창률은 시각 표현이며 개방 충격·노즈 분리를 계산하지 않습니다."
              : ground
                ? "지면 도달 · 착지 시점 기록 확인. 충격·착륙다리·모터 정지 동역학은 포함하지 않습니다."
                : `회수 시간 R + ${(row.recovery_t_s ?? row.t_s).toFixed(1)}s · 센서 상태를 확인하며 ${result.config.mode === "target" ? "목표 PD를 반복 계산합니다." : "선택한 조향 모드를 실행합니다."}`;
  for (const b of $("mission-events").children)
    b.setAttribute(
      "aria-current",
      String(fullMission() && b.dataset.phase === phase),
    );
  const valid = row.guidance_valid && !beforeRecovery && !ground;
  $("feedback-status").textContent = beforeRecovery
    ? "회수 제어 시작 전"
    : ground
      ? "착지 · 제어 종료"
      : phase === "TIME_LIMIT"
        ? "미착지 · 계산 종료"
        : result.config.mode !== "target"
          ? "목표 추종 꺼짐"
          : !valid
            ? "센서 대기 · 중립"
            : row.estimated_upwind_unreachable
              ? "맞바람 · 도달 불가 추정"
              : row.command_saturated
                ? "추종 중 · 당김 제한 도달"
                : "추종 중 · PD 갱신";
  $("feedback-gps").textContent = row.gps_valid
    ? `E ${row.gps_east_m.toFixed(1)} / N ${row.gps_north_m.toFixed(1)} m`
    : "GPS 대기 / 시간초과";
  $("feedback-age").textContent =
    `GPS 나이 ${row.gps_age_s < 0 ? "—" : row.gps_age_s.toFixed(2) + " s"} · 제어 ${row.control_tick}회`;
  $("feedback-error").textContent = valid
    ? `방위 오차 ${row.heading_error_deg.toFixed(1)}°`
    : "목표 방위 계산 대기";
  $("feedback-range").textContent =
    `실제 거리 ${row.target_distance_m.toFixed(1)} m · 추정 ${valid ? row.estimated_distance_m.toFixed(1) + " m" : "—"}`;
  $("feedback-pd").textContent = valid
    ? `${row.desired_yaw_rate_deg_s.toFixed(1)} °/s`
    : "—";
  $("feedback-terms").textContent = valid
    ? `P ${((row.p_term_rad_s * 180) / Math.PI).toFixed(1)} + D ${((row.d_term_rad_s * 180) / Math.PI).toFixed(1)} °/s`
    : "센서 유효 상태 확인";
  $("feedback-brake").textContent =
    `${(row.left_brake_m * 1000).toFixed(1)} / ${(row.right_brake_m * 1000).toFixed(1)} mm`;
  $("feedback-request").textContent =
    result.config.mode === "target"
      ? `지령 ${(row.requested_left_brake_m * 1000).toFixed(1)} / ${(row.requested_right_brake_m * 1000).toFixed(1)} mm`
      : `각도 지령 ${row.left_command_deg.toFixed(0)} / ${row.right_command_deg.toFixed(0)}°`;
  const state = [
    [
      "자세 근거",
      row.attitude_source === "ORK_SAVED_AXIS"
        ? "ORK 저장 자세 · 축회전 미복원"
        : row.attitude_source === "VERTICAL_1D_ASSUMPTION"
          ? "1D 수직 가정 · 자세 미계산"
          : row.attitude_source === "ILLUSTRATIVE_ATTITUDE_BLEND"
            ? "전개 자세 보간 · 가정"
            : "회수 뱅크 근사",
    ],
    [
      "모터 속도 · 좌 / 우",
      `${row.left_velocity_deg_s.toFixed(1)} / ${row.right_velocity_deg_s.toFixed(1)} °/s`,
    ],
    ["미션 단계", row.phase_label || "회수 비행"],
    [
      "수직 속도 / 추력",
      `${(row.vertical_velocity_mps ?? -row.sink_mps).toFixed(2)} m/s / ${(row.thrust_n || 0).toFixed(1)} N`,
    ],
    [
      "GPS 진행 방향 / 추정 방위",
      row.gps_valid && row.imu_valid
        ? `${row.gps_course_deg.toFixed(1)}° / ${row.sensor_heading_deg.toFixed(1)}°`
        : "—",
    ],
    [
      "바람 · 동 / 북",
      `${row.wind_east_mps.toFixed(2)} / ${row.wind_north_mps.toFixed(2)} m/s`,
    ],
    ["고도", row.altitude_m.toFixed(1) + " m"],
    [
      "방위 / 선회율",
      (((row.heading_deg % 360) + 360) % 360).toFixed(1) +
        "° / " +
        row.yaw_rate_deg_s.toFixed(1) +
        "°/s",
    ],
    [
      "대기 속도 / 하강률",
      beforeRecovery
        ? "회수 모델 시작 전"
        : row.airspeed_mps.toFixed(2) +
          " / " +
          row.sink_mps.toFixed(2) +
          " m/s",
    ],
    [
      "줄 장력 · 좌 / 우",
      row.left_tension_n.toFixed(1) +
        " / " +
        row.right_tension_n.toFixed(1) +
        " N",
    ],
    [
      "토크 · 좌 / 우",
      row.left_torque_nm.toFixed(3) +
        " / " +
        row.right_torque_nm.toFixed(3) +
        " N·m",
    ],
    [
      "구동 상태",
      row.left_overloaded || row.right_overloaded
        ? "토크 한계 초과"
        : "설정 토크 범위 이내",
    ],
  ];
  $("state").replaceChildren(
    ...state.map(([k, v]) => {
      const el = document.createElement("div"),
        dt = document.createElement("dt"),
        dd = document.createElement("dd");
      dt.textContent = k;
      dd.textContent = v;
      el.append(dt, dd);
      return el;
    }),
  );
}
async function run(event) {
  event?.preventDefault();
  playing = false;
  $("play").textContent = "▶ 재생";
  $("run").disabled = true;
  setStatus("물리 모델 계산 중…");
  try {
    const response = await fetch("/api/simulate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(readConfig()),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error);
    result = data;
    const s = data.summary;
    $("voltage-status").textContent = data.motor_voltage_matches_performance
      ? `공급 설정과 성능 자료 전압 일치 · ${data.config.servo_supply_voltage_v} V. 실제 성능 보정 여부는 자료를 확인하세요.`
      : `전압 불일치: 공급 설정 ${data.config.servo_supply_voltage_v} V / 토크·속도 자료 ${data.config.motor_performance_voltage_v} V. 아래 결과는 ${data.config.motor_performance_voltage_v} V 성능을 사용한 참조 계산이며, ${data.config.servo_supply_voltage_v} V 실제 성능을 예측한 것이 아닙니다.`;
    $("metric-pull").textContent =
      s.max_left_brake_mm.toFixed(1) +
      " / " +
      s.max_right_brake_mm.toFixed(1) +
      " mm";
    $("metric-turn").textContent = s.max_yaw_rate_deg_s.toFixed(2) + " °/s";
    $("metric-distance").textContent = s.target_error_m.toFixed(1) + " m";
    $("endpoint-label").textContent = s.landed
      ? "목표까지 착지 오차"
      : "목표까지 종료점 거리 (미착지)";
    missionTrajectory = assembleMission(data);
    neutralMissionTrajectory = assembleMission(data.neutral);
    missionNavigation();
    const g = data.rocket_geometry,
      nose = g.parts.find((p) => p.kind === "nosecone"),
      body = g.parts.find((p) => p.kind === "bodytube"),
      fins = g.parts.find((p) => p.kind === "trapezoidfinset");
    $("geometry-caption").textContent =
      `ORK 외형: 길이 ${(g.length_m * 1000).toFixed(1)} mm · 지름 ${(body.radius_m * 2000).toFixed(0)} mm · 타원형 노즈 · 사다리꼴 핀 ${fins.count}개. 캐노피: 링크 제품 참고 외형.`;
    $("geometry-details").textContent =
      `OpenRocket 파일에서 직접 읽은 치수: 노즈 ${(nose.length_m * 1000).toFixed(0)} mm, 몸통 ${(body.length_m * 1000).toFixed(0)} mm, 보트테일 ${(g.parts.find((p) => p.kind === "transition").length_m * 1000).toFixed(3)} mm. 핀 루트 ${(fins.root_chord_m * 1000).toFixed(2)} mm / 팁 ${(fins.tip_chord_m * 1000).toFixed(2)} mm / 높이 ${(fins.span_m * 1000).toFixed(2)} mm / 후퇴 ${(fins.sweep_m * 1000).toFixed(2)} mm. 핀 단면은 두께를 보존한 평판 근사입니다.`;
    $("comparison").textContent =
      `${s.control_updates.toLocaleString()}회 제어 계산 · 목표 최근접 ${s.closest_target_distance_m.toFixed(1)} m · 무조향 ${data.neutral.summary.landed ? "착지" : "종료"} 오차 ${data.neutral.summary.target_error_m.toFixed(1)} m · 무조향 대비 ${s.landed && data.neutral.summary.landed ? "착지" : "종료"}점 차이 ${s.endpoint_difference_from_neutral_m.toFixed(1)} m. ${s.landed ? (s.landed_in_target ? "이 시나리오에서 목표 반경 내 착지." : "이 시나리오에서 목표 반경 밖 착지.") : "계산 시간 안에 착지하지 않았습니다."}`;
    const aa = data.mission.ascent_attitude_summary;
    $("physics-status").textContent =
      (aa.source === "ORK_SAVED_AXIS"
        ? `상승: ORK 저장 위치·기울기·각속도 재생. 최고점 횡이동 ${aa.horizontal_at_apogee_m.toFixed(1)} m / 최대 기울기 ${aa.max_tilt_deg.toFixed(1)}°. 상승 바람은 ORK 저장값입니다.`
        : "상승: ENG 기반 1D 수직 모델. 횡풍·자세 운동은 계산하지 않습니다.") +
      ` 회수: 모터 지연 ${data.config.servo_latency_s}s, 가감속 ${data.config.servo_accel_deg_s2}°/s², 센서 편향/노이즈, ${data.config.wind_inertia_enabled ? "변동 바람의 관성항 포함" : "바람 이류만 적용"}. 이 캐노피의 실제 조향 가능성은 미검증입니다.`;
    configureView();
    $("track-caption").textContent =
      `점화 T=0 → 최고점 ${s.apogee_time_s.toFixed(2)} s / ${s.apogee_m.toFixed(1)} m → 회수 시작 T+${data.mission.recovery_start_s.toFixed(2)} s → ${s.landed ? "착지 T+" + data.mission.touchdown_s.toFixed(1) + " s" : "미착지 종료"}. 회수 설정 시간은 별도 R=0 기준이며 전개 전환은 가정입니다.`;
    $("source-details").textContent = JSON.stringify(
      {
        model_assessment: data.model_assessment,
        servo_reference: data.servo_reference,
        motor_voltage_matches_performance:
          data.motor_voltage_matches_performance,
        uses_reference_torque_speed: data.uses_reference_torque_speed,
        initialization_note: data.initialization_note,
        warnings: data.warnings,
        config: data.config,
      },
      null,
      2,
    );
    setStatus("계산 완료 · 발사부터 착지까지 미션을 재생합니다.");
    playing = true;
    lastFrame = null;
    $("play").textContent = "Ⅱ 일시정지";
    draw();
  } catch (error) {
    setStatus(
      "계산 실패: " + error.message + " · 표시 중인 그래프는 이전 결과입니다.",
      "error",
    );
  } finally {
    $("run").disabled = false;
  }
}
function download(name, data, type) {
  const a = document.createElement("a"),
    url = URL.createObjectURL(new Blob([data], { type }));
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$("controls").addEventListener("submit", run);
$("controls").addEventListener("input", () => {
  setStatus("설정이 변경되었습니다. 실행하면 결과에 반영됩니다.", "stale");
  if ($("sensitivity-table").hasChildNodes())
    $("sensitivity-status").textContent =
      "설정 변경됨 · 이전 입력의 비교입니다.";
  if ($("sweep-table").hasChildNodes())
    $("sweep-status").textContent =
      "설정 변경됨 · 아래 표는 이전 계산입니다. 다시 비교 계산하세요.";
});
$("scrub").addEventListener("input", () => {
  playing = false;
  $("play").textContent = "▶ 재생";
  playTime = result ? viewRows()[currentIndex()].t_s : 0;
  draw();
});
$("play").addEventListener("click", () => {
  if (!result) return;
  if (currentIndex() === viewRows().length - 1) {
    playTime = viewRows()[0].t_s;
    $("scrub").value = 0;
  }
  playing = !playing;
  $("play").textContent = playing ? "Ⅱ 일시정지" : "▶ 재생";
  lastFrame = null;
});
$("view-scope").addEventListener("change", () => {
  playing = false;
  $("play").textContent = "▶ 재생";
  configureView();
});
$("mission-restart").addEventListener("click", () => {
  if (!result) return;
  $("view-scope").value = "mission";
  configureView();
  playing = true;
  $("play").textContent = "Ⅱ 일시정지";
});
$("mission-csv").addEventListener("click", () => {
  if (!result) return;
  const keys = Object.keys(missionTrajectory[0]);
  download(
    "tusi-mission.csv",
    [
      keys.join(","),
      ...missionTrajectory.map((r) => keys.map((k) => r[k]).join(",")),
    ].join("\n"),
    "text/csv",
  );
});
$("plot-kind").addEventListener("change", draw);
$("plot-window").addEventListener("change", draw);
$("json").addEventListener("click", () => {
  if (result)
    download(
      "tusi-result.json",
      JSON.stringify(result, null, 2),
      "application/json",
    );
});
$("config-download").addEventListener("click", () =>
  download(
    "tusi-config.json",
    JSON.stringify(readConfig(), null, 2),
    "application/json",
  ),
);
$("csv").addEventListener("click", () => {
  if (!result) return;
  const keys = Object.keys(result.trajectory[0]);
  download(
    "tusi-trajectory.csv",
    [
      keys.join(","),
      ...result.trajectory.map((r) => keys.map((k) => r[k]).join(",")),
    ].join("\n"),
    "text/csv",
  );
});
$("sweep").addEventListener("click", async () => {
  const button = $("sweep");
  button.disabled = true;
  $("sweep-status").textContent = "각도별 물리 응답 계산 중…";
  const requestConfig = readConfig();
  try {
    const response = await fetch("/api/sweep", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(requestConfig),
    });
    const data = await response.json();
    if (!response.ok) throw Error(data.error);
    const table = document.createElement("table");
    const head = document.createElement("tr");
    for (const title of [
      "회전각",
      "최대 유효 당김",
      "최대 선회율",
      "동 / 북 차이",
      "위치 차이",
      "종료 시각",
    ]) {
      const th = document.createElement("th");
      th.textContent = title;
      head.append(th);
    }
    table.append(head);
    for (const r of data.rows) {
      const tr = document.createElement("tr");
      for (const value of [
        r.command_deg.toFixed(0) + "°",
        r.brake_mm.toFixed(1) + " mm",
        r.yaw_deg_s.toFixed(2) + " °/s",
        r.delta_east_m.toFixed(1) + " / " + r.delta_north_m.toFixed(1) + " m",
        r.displacement_from_neutral_m.toFixed(1) + " m",
        r.time_s.toFixed(1) + " s" + (r.landed ? " 착지" : ""),
      ]) {
        const td = document.createElement("td");
        td.textContent = value;
        tr.append(td);
      }
      table.append(tr);
    }
    $("sweep-table").replaceChildren(table);
    $("sweep-status").textContent =
      `수동 입력 ${data.config.command_start_s}~${data.config.command_end_s} s · 미보정 모델의 종료점 비교입니다. ` +
      (JSON.stringify(requestConfig) !== JSON.stringify(readConfig())
        ? "계산 중 설정이 변경됐습니다. 다시 비교 계산하세요."
        : "");
  } catch (e) {
    $("sweep-status").textContent =
      "비교 실패: " + e.message + " · 기존 표는 이전 결과입니다.";
  } finally {
    button.disabled = false;
  }
});
$("sensitivity").addEventListener("click", async () => {
  const button = $("sensitivity");
  button.disabled = true;
  const cfg = readConfig();
  $("sensitivity-status").textContent = "8가지 가정을 계산 중…";
  try {
    const response = await fetch("/api/sensitivity", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(cfg),
    });
    const data = await response.json();
    if (!response.ok) throw Error(data.error);
    const table = document.createElement("table"),
      head = document.createElement("tr");
    for (const title of [
      "가정",
      "착지/종료 오차",
      "최대 당김",
      "최대 토크",
      "지령 포화",
      "판정",
    ]) {
      const th = document.createElement("th");
      th.textContent = title;
      head.append(th);
    }
    table.append(head);
    for (const r of data.rows) {
      const tr = document.createElement("tr");
      const values =
        r.error_m === undefined
          ? [r.label, "—", "—", "—", "—", r.status]
          : [
              r.label,
              r.error_m.toFixed(1) + " m",
              r.brake_mm.toFixed(1) + " mm",
              r.torque_nm.toFixed(3) + " N·m",
              (r.saturation_fraction * 100).toFixed(0) + "%",
              r.validity_exceeded
                ? "보정 범위 초과"
                : r.landed
                  ? "가정 내 착지"
                  : "미착지",
            ];
      for (const value of values) {
        const td = document.createElement("td");
        td.textContent = value;
        tr.append(td);
      }
      tr.title = JSON.stringify(r.overrides);
      table.append(tr);
    }
    $("sensitivity-table").replaceChildren(table);
    $("sensitivity-status").textContent =
      data.note +
      (JSON.stringify(cfg) !== JSON.stringify(readConfig())
        ? " 설정이 변경됐습니다. 표는 이전 입력 기준입니다."
        : "");
  } catch (e) {
    $("sensitivity-status").textContent = "계산 실패: " + e.message;
  } finally {
    button.disabled = false;
  }
});
$("control-csv").addEventListener("click", () => {
  if (!result) return;
  const keys = Object.keys(result.control_log[0]);
  download(
    "tusi-control-log.csv",
    [
      keys.join(","),
      ...result.control_log.map((r) => keys.map((k) => r[k]).join(",")),
    ].join("\n"),
    "text/csv",
  );
});
for (const mode of ["follow", "rocket", "launch", "overview"])
  $("camera-" + mode).addEventListener("click", () => {
    scene?.setMode(mode);
    for (const m of ["follow", "rocket", "launch", "overview"])
      $("camera-" + m).setAttribute("aria-pressed", String(m === mode));
    $("scene-mode").textContent =
      mode === "follow"
        ? "따라가기 · ORK 로켓 / 제품 참고 캐노피"
        : mode === "rocket"
          ? "로켓 확대 · ORK 부품 치수"
          : mode === "launch"
            ? "상승 궤적 · 고정 카메라 / 기체 8배"
            : "전체 경로 · 기체 8배 확대";
  });
for (const button of document.querySelectorAll("[data-preset]"))
  button.addEventListener("click", () => {
    if ($("run").disabled || !configDefaults) return;
    const name = button.dataset.preset;
    fillConfig({
      ...configDefaults,
      ...(name === "gust"
        ? { gust_std_mps: 1.5 }
        : name === "dropout"
          ? { gps_dropout_start_s: 20, gps_dropout_end_s: 26 }
          : name === "manual"
            ? {
                mode: "manual",
                gust_std_mps: 0,
                wind_east_mps: 0,
                wind_north_mps: 0,
                wind_shear_east_mps_100m: 0,
              }
            : {}),
    });
    run();
  });
$("config-upload").addEventListener("change", async (event) => {
  try {
    const file = event.target.files[0];
    if (!file) return;
    const cfg = JSON.parse(await file.text());
    if (
      !cfg ||
      typeof cfg !== "object" ||
      Array.isArray(cfg) ||
      Object.keys(cfg).some((k) => !(k in configDefaults))
    )
      throw new Error("알 수 없는 설정 항목");
    fillConfig({ ...configDefaults, ...cfg });
    setStatus("설정을 불러왔습니다. 실행해 반영하세요.", "stale");
  } catch (e) {
    setStatus("설정 오류: " + e.message, "error");
  }
});
window.addEventListener("resize", draw);
let lastDraw = 0;
function animate(timestamp) {
  if (playing && result) {
    const rows = viewRows(),
      row = rows[currentIndex()];
    const slow =
      fullMission() &&
      $("phase-speed").checked &&
      row.phase !== "GUIDED_DESCENT";
    if (lastFrame !== null)
      playTime +=
        Math.min(0.25, (timestamp - lastFrame) / 1000) *
        (slow ? 1 : Number($("speed").value));
    lastFrame = timestamp;
    if (playTime >= rows.at(-1).t_s) {
      if ($("loop").checked) {
        playTime = rows[0].t_s;
      } else {
        playTime = rows.at(-1).t_s;
        playing = false;
        $("play").textContent = "▶ 재생";
      }
    }
    $("scrub").value = indexAt(playTime);
    if (timestamp - lastDraw > 50) {
      draw();
      lastDraw = timestamp;
    } else scene?.update(rows[currentIndex()]);
  }
  scene?.render();
  requestAnimationFrame(animate);
}
async function init() {
  try {
    const { FlightScene } = await import("./scene.js");
    scene = new FlightScene($("scene"));
  } catch (error) {
    $("scene").textContent =
      "3D 표시를 시작할 수 없습니다. WebGL 지원 브라우저에서 열어주세요. 아래 2D 경로와 제어 로그는 계속 사용할 수 있습니다.";
    console.error(error);
  }
  try {
    const response = await fetch("/api/config");
    if (!response.ok) throw new Error("입력 파일을 읽을 수 없습니다.");
    const {
      config,
      sources,
      servo_reference: servo,
      canopy_reference,
    } = await response.json();
    configDefaults = config;
    if (scene) scene.canopyReference = canopy_reference;
    $("servo-reference").textContent =
      `${servo.model} · 회로도 설계 전압 ${servo.design_supply_voltage_v} V. 제조사 성능 자료는 ${servo.reference_voltage_v} V: ${servo.stall_torque_kgf_cm} kgf·cm / ${servo.no_load_speed_rpm} rpm. TTL 시리얼 버스 · 위치 0~4095. 위치 제어 모드를 가정합니다.`;
    for (const [group, items] of Object.entries(fields))
      for (const [name, label, step] of items) {
        const wrapper = document.createElement("label"),
          input = document.createElement("input");
        wrapper.textContent = label;
        input.name = name;
        input.type = "number";
        input.step = step;
        input.required = true;
        wrapper.append(input);
        $(group + "-fields").append(wrapper);
      }
    fillConfig(config);
    $("source-stats").innerHTML =
      `<div>ORK 회수 질량 <strong>${sources.saved_recovery_mass_kg.toFixed(3)} <small>kg</small></strong></div><div>ORK 최고 고도 <strong>${sources.saved_apogee_m.toFixed(1)} <small>m</small></strong></div>`;
    await run();
    requestAnimationFrame(animate);
  } catch (error) {
    setStatus(error.message, "error");
  }
}
init();
