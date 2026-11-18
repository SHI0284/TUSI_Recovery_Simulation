// 2D 궤적·드럼·응답 그래프. 시뮬레이션을 실행하지 않고 계산된 행을 그린다.
const $ = (id) => document.getElementById(id);
const colors = {
  active: "#67dfc3",
  left: "#b19cff",
  neutral: "#8996ab",
  grid: "#283547",
  text: "#a6b6c9",
};
function setupCanvas(id) {
  const canvas = $(id),
    width = canvas.clientWidth,
    height = canvas.clientHeight;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  ctx.font = "11px system-ui";
  ctx.fillStyle = colors.text;
  return { ctx, w: width, h: height };
}
function line(ctx, points, color, width = 2, dash = []) {
  if (!points.length) return;
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  points.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
  ctx.setLineDash([]);
}
export function drawTrack(row, result, active, neutral, currentIndex) {
  const { ctx, w, h } = setupCanvas("trajectory");
  const c = result.config;
  const all = [
    active,
    neutral,
    [
      { east_m: 0, north_m: 0 },
      {
        east_m: c.target_east_m - c.target_radius_m,
        north_m: c.target_north_m - c.target_radius_m,
      },
      {
        east_m: c.target_east_m + c.target_radius_m,
        north_m: c.target_north_m + c.target_radius_m,
      },
    ],
  ];
  let minX = Infinity,
    maxX = -Infinity,
    minY = Infinity,
    maxY = -Infinity;
  all.forEach((rows) =>
    rows.forEach((r) => {
      minX = Math.min(minX, r.east_m);
      maxX = Math.max(maxX, r.east_m);
      minY = Math.min(minY, r.north_m);
      maxY = Math.max(maxY, r.north_m);
    }),
  );
  const scale = Math.min(
    (w - 105) / Math.max(30, maxX - minX),
    (h - 65) / Math.max(30, maxY - minY),
  );
  const midX = (maxX + minX) / 2,
    midY = (maxY + minY) / 2;
  const map = (x, y) => [
    w / 2 + scale * (x - midX),
    h / 2 - scale * (y - midY),
  ];
  const tick =
    10 ** Math.floor(Math.log10(Math.max(30, maxX - minX, maxY - minY)));
  for (let x = Math.floor(minX / tick) * tick; x <= maxX + tick; x += tick) {
    const px = map(x, 0)[0];
    if (px < 40 || px > w - 20) continue;
    line(
      ctx,
      [
        [px, 20],
        [px, h - 30],
      ],
      colors.grid,
      1,
    );
    ctx.fillText(x.toFixed(0), px + 3, h - 15);
  }
  for (let y = Math.floor(minY / tick) * tick; y <= maxY + tick; y += tick) {
    const py = map(0, y)[1];
    if (py < 20 || py > h - 30) continue;
    line(
      ctx,
      [
        [40, py],
        [w - 20, py],
      ],
      colors.grid,
      1,
    );
    ctx.fillText(y.toFixed(0), 3, py + 4);
  }
  ctx.fillText("N / 북 (m)", 5, 12);
  ctx.fillText("E / 동 (m)", w - 76, h - 1);
  const downsample = (rows) =>
    rows
      .filter((_, i) => i % 8 === 0 || i === rows.length - 1)
      .map((r) => map(r.east_m, r.north_m));
  line(ctx, downsample(neutral), colors.neutral, 1.5, [5, 5]);
  line(ctx, downsample(active), "#294f49", 2);
  line(ctx, downsample(active.slice(0, currentIndex + 1)), colors.active, 2.5);
  for (const [r, text, color] of [
    [active[0], "시작", colors.text],
    [active.at(-1), result.summary.landed ? "착지" : "종료", colors.active],
  ]) {
    const [x, y] = map(r.east_m, r.north_m);
    ctx.beginPath();
    ctx.arc(x, y, 3, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.fillText(text, Math.min(w - 40, x + 7), Math.max(12, y - 7));
  }
  const [tx, ty] = map(c.target_east_m, c.target_north_m);
  ctx.strokeStyle = "#ffce76";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(tx, ty, c.target_radius_m * scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#ffce76";
  ctx.fillText("목표", tx + 5, ty - 5);
  if (row.gps_valid) {
    const [gx, gy] = map(row.gps_east_m, row.gps_north_m);
    ctx.fillStyle = colors.left;
    ctx.beginPath();
    ctx.arc(gx, gy, 3, 0, Math.PI * 2);
    ctx.fill();
  }
  const [x, y] = map(row.east_m, row.north_m);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate((row.heading_deg * Math.PI) / 180);
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(6, 7);
  ctx.lineTo(0, 4);
  ctx.lineTo(-6, 7);
  ctx.closePath();
  ctx.fillStyle = colors.active;
  ctx.fill();
  ctx.restore();
}
export function drawMechanism(row, result) {
  const { ctx, w, h } = setupCanvas("mechanism");
  for (const [side, x, color, label] of [
    ["left", w * 0.25, colors.left, "LEFT · 좌"],
    ["right", w * 0.75, colors.active, "RIGHT · 우"],
  ]) {
    const y = 63,
      r = 25,
      angle = (row[side + "_angle_deg"] * Math.PI) / 180;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.stroke();
    line(
      ctx,
      [
        [x, y],
        [x + r * Math.sin(angle), y - r * Math.cos(angle)],
      ],
      color,
      2,
    );
    line(
      ctx,
      [
        [x + r, y],
        [x + r, 125 - row[side + "_brake_m"] * 500],
      ],
      color,
      2,
    );
    ctx.fillStyle = colors.text;
    ctx.textAlign = "center";
    ctx.fillText(label, x, 14);
    ctx.fillStyle = color;
    ctx.font = "16px ui-monospace";
    ctx.fillText(row[side + "_angle_deg"].toFixed(1) + "°", x, 155);
    ctx.font = "11px system-ui";
    ctx.fillStyle = colors.text;
    ctx.fillText(
      "권취 " + (row[side + "_takeup_m"] * 1000).toFixed(1) + " mm",
      x,
      176,
    );
    ctx.fillText(
      "유효 " + (row[side + "_brake_m"] * 1000).toFixed(1) + " mm",
      x,
      194,
    );
  }
  ctx.textAlign = "left";
}
export function drawHistory(row, result, rows, fullMission) {
  const { ctx, w, h } = setupCanvas("history");
  const kinds = {
    tilt: [["body_tilt_deg", "수직 대비 기울기", colors.active, 1]],
    rates: [
      ["body_pitch_rate_deg_s", "ORK 피치", colors.active, 1],
      ["body_yaw_rate_deg_s", "ORK 요", colors.left, 1],
      ["body_roll_rate_deg_s", "ORK 롤", colors.neutral, 1],
    ],
    motor: [
      ["right_command_deg", "우 지령", colors.neutral, 1],
      ["right_applied_target_deg", "우 수신", colors.left, 1],
      ["right_angle_deg", "우 실제", colors.active, 1],
    ],
    brake: [
      ["left_brake_m", "실제 좌", colors.left, 1000],
      ["right_brake_m", "실제 우", colors.active, 1000],
      ...(result.config.mode === "target"
        ? [
            ["requested_left_brake_m", "지령 좌", "#7965ae", 1000],
            ["requested_right_brake_m", "지령 우", "#389780", 1000],
          ]
        : []),
    ],
    error: [["heading_error_deg", "방위 오차", colors.active, 1]],
    distance: [["target_distance_m", "실제 거리", colors.active, 1]],
    wind: [
      ["wind_east_mps", "실제 동", colors.active, 1],
      ["estimated_wind_east_mps", "추정 동", "#389780", 1],
      ["wind_north_mps", "실제 북", colors.left, 1],
      ["estimated_wind_north_mps", "추정 북", "#7965ae", 1],
    ],
    yaw: [["yaw_rate_deg_s", "선회율", colors.active, 1]],
    altitude: [["altitude_m", "고도", colors.active, 1]],
    torque: [
      ["left_torque_nm", "좌", colors.left, 1],
      ["right_torque_nm", "우", colors.active, 1],
    ],
  };
  const series = kinds[$("plot-kind").value];
  const widthS = Number($("plot-window").value),
    start = widthS
      ? Math.max(
          rows[0].t_s,
          Math.min(row.t_s - widthS / 2, rows.at(-1).t_s - widthS),
        )
      : rows[0].t_s,
    end = widthS ? Math.min(rows.at(-1).t_s, start + widthS) : rows.at(-1).t_s;
  const plotted = rows
    .filter((r) => r.t_s >= start && r.t_s <= end)
    .filter((r) =>
      $("plot-kind").value === "rates"
        ? r.angular_rates_available
        : !fullMission ||
          ["altitude", "tilt"].includes($("plot-kind").value) ||
          r.recovery_t_s >= 0,
    );
  let lo = 0,
    hi = 0.001;
  for (const [key, , , factor] of series)
    for (const r of plotted) {
      lo = Math.min(lo, r[key] * factor);
      hi = Math.max(hi, r[key] * factor);
    }
  const map = (t, v) => [
    48 + ((t - start) / Math.max(0.001, end - start)) * (w - 65),
    h - 27 - ((v - lo) / (hi - lo)) * (h - 50),
  ];
  for (let i = 0; i < 5; i++) {
    const v = lo + ((hi - lo) * i) / 4,
      y = map(0, v)[1];
    line(
      ctx,
      [
        [48, y],
        [w - 17, y],
      ],
      colors.grid,
      1,
    );
    ctx.fillText(v.toFixed(hi < 1 ? 3 : 1), 2, y + 4);
  }
  for (let i = 0; i < 5; i++) {
    const t = start + ((end - start) * i) / 4;
    ctx.fillText(t.toFixed(widthS ? 1 : 0) + " s", map(t, 0)[0] - 10, h - 5);
  }
  series.forEach(([key, label, color, factor], i) => {
    line(
      ctx,
      plotted
        .filter((_, j) => widthS || j % 5 === 0 || j === plotted.length - 1)
        .map((r) => map(r.t_s, r[key] * factor)),
      color,
    );
    ctx.fillStyle = color;
    ctx.fillText(label, 60 + i * 65, 12);
  });
  const x = map(row.t_s, 0)[0];
  line(
    ctx,
    [
      [x, 22],
      [x, h - 27],
    ],
    "#e0e8f3",
    1,
    [3, 4],
  );
}
