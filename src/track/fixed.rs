//! 内嵌固定真实轨迹。
//!
//! 资产 `assets/fixed_route.json` 启用后，跑步轨迹改为沿一条真实路线（用户人工跑步
//! 导出的轨迹）行进：坐标系归一到 BD-09，刚体变换（旋转 + 等比缩放 + 平移）贴合到
//! 本次任务打卡点，稀疏段重采样，闭环直接循环、开放路径往返；离打卡点过远时插入
//! 平滑绕行弯。速度曲线、步频、采样间隔、哨兵与异常点全部沿用随机生成器逻辑。
//!
//! 文件格式（JSON）：
//! `{"enabled":true,"name":"…","coordType":"wgs84|gcj02|bd09","points":[[lat,lng],…]}`

use super::geom::{build_arcs, round_to, to_bd, MET_PER_DEG_LAT, MET_PER_DEG_LNG, PointRing};
use serde_json::Value;

const ROUTE_ASSET: &[u8] = include_bytes!("../../assets/fixed_route.json");

/// 打卡点吸附半径内不强制绕行（吸附会把最近点精确落到打卡点）。
const SNAP_RADIUS_M: f64 = 35.0;
/// 绕行弯两侧保留的原路径长度。
const DETOUR_SHOULDER_M: f64 = 25.0;
/// 重采样目标间距（米）。
const RESAMPLE_STEP_M: f64 = 3.0;
/// 首尾距离 ≤ 此值视为闭环（否则往返）。
const CLOSE_GAP_M: f64 = 40.0;

pub struct FixedRoute {
    pub name: String,
    pub coord_type: String,
    pub points: Vec<(f64, f64)>,
}

pub struct PreparedRing {
    pub ring: PointRing,
    pub name: String,
    pub src_points: usize,
    pub path_len_m: f64,
    pub closed: bool,
    pub max_cp_residual_m: f64,
    pub detours: usize,
}

/// 读取内嵌固定轨迹；未启用/为空返回 None。
pub fn load_embedded_route() -> Result<Option<FixedRoute>, String> {
    parse_route_json(ROUTE_ASSET)
}

fn parse_route_json(bytes: &[u8]) -> Result<Option<FixedRoute>, String> {
    let v: Value = serde_json::from_slice(bytes).map_err(|e| format!("固定轨迹 JSON 解析失败: {e}"))?;
    if !v.get("enabled").and_then(Value::as_bool).unwrap_or(false) {
        return Ok(None);
    }
    let name = v
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or("固定轨迹")
        .to_string();
    let coord_type = v
        .get("coordType")
        .and_then(Value::as_str)
        .unwrap_or("wgs84")
        .to_ascii_lowercase();
    if !matches!(coord_type.as_str(), "wgs84" | "gcj02" | "bd09") {
        return Err(format!("固定轨迹 coordType 无效: {coord_type}"));
    }
    let arr = v
        .get("points")
        .and_then(Value::as_array)
        .ok_or("固定轨迹缺少 points 数组")?;
    let mut points = Vec::with_capacity(arr.len());
    for item in arr {
        let pair = item.as_array().ok_or("固定轨迹 points 元素必须是 [lat, lng]")?;
        if pair.len() != 2 {
            return Err("固定轨迹 points 元素必须是 [lat, lng]".into());
        }
        let (lat, lng) = (
            pair[0].as_f64().ok_or("固定轨迹纬度不是数字")?,
            pair[1].as_f64().ok_or("固定轨迹经度不是数字")?,
        );
        if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lng) {
            return Err(format!("固定轨迹坐标越界: ({lat}, {lng})"));
        }
        points.push((lat, lng));
    }
    if points.len() < 8 {
        return Err(format!("固定轨迹点数不足（{} < 8）", points.len()));
    }
    Ok(Some(FixedRoute { name, coord_type, points }))
}

/// 生产统一入口：内嵌固定轨迹可用时返回贴合后的环，否则 None。日志行追加到 out。
pub fn ring_for_run(points_bd: &[(f64, f64)], out: &mut Vec<String>) -> Option<PointRing> {
    let route = match load_embedded_route() {
        Ok(Some(r)) => r,
        Ok(None) => return None,
        Err(e) => {
            out.push(format!("⚠ [track] 固定轨迹读取失败：{e}；回退打卡点拟合环"));
            return None;
        }
    };
    match prepare_ring(points_bd, &route) {
        Ok(p) => {
            out.push(format!(
                "√ [track] 固定轨迹「{}」：{} 源点 → 路径 {:.0}m（{}），贴合残差 {:.0}m{}",
                p.name,
                p.src_points,
                p.path_len_m,
                if p.closed { "闭环循环" } else { "往返" },
                p.max_cp_residual_m,
                if p.detours > 0 {
                    format!("，{} 处打卡绕行弯", p.detours)
                } else {
                    String::new()
                },
            ));
            Some(p.ring)
        }
        Err(e) => {
            out.push(format!("⚠ [track] 固定轨迹不可用：{e}；回退打卡点拟合环"));
            None
        }
    }
}

/// 真实轨迹 → 贴合打卡点的闭合环。
pub fn prepare_ring(cps_bd: &[(f64, f64)], route: &FixedRoute) -> Result<PreparedRing, String> {
    if cps_bd.is_empty() {
        return Err("打卡点为空".into());
    }
    let src = to_bd09(&route.points, &route.coord_type)?;
    if src.len() < 8 {
        return Err(format!("固定轨迹点数不足（{} < 8）", src.len()));
    }
    // 平面化：真实轨迹以自身质心、打卡点以几何中位点为原点（米）
    let rc = centroid(&src);
    let plane: Vec<(f64, f64)> = src
        .iter()
        .map(|p| ((p.1 - rc.1) * MET_PER_DEG_LNG, (p.0 - rc.0) * MET_PER_DEG_LAT))
        .collect();
    let cc = robust_center(cps_bd);
    let cps: Vec<(f64, f64)> = cps_bd
        .iter()
        .map(|p| ((p.1 - cc.1) * MET_PER_DEG_LNG, (p.0 - cc.0) * MET_PER_DEG_LAT))
        .collect();

    // 刚体贴合：搜索旋转 + 等比缩放 + 稳健平移，最小化打卡点到路径的截断平方距离和
    // （plane 与 cps 各自以自身质心为原点，故对齐平移恒为零，无需再加质心）
    let (theta, scale, shift) = fit_transform(&plane, &cps);
    let (st, ct) = theta.sin_cos();
    let mut fitted: Vec<(f64, f64)> = plane
        .iter()
        .map(|p| {
            (
                shift.0 + scale * (ct * p.0 - st * p.1),
                shift.1 + scale * (st * p.0 + ct * p.1),
            )
        })
        .collect();
    fitted.dedup_by(|a, b| dist2(*a, *b) < 1e-6);
    let path_len = polyline_len(&fitted);
    if path_len < 50.0 {
        return Err(format!("固定轨迹路径过短（{path_len:.0}m < 50m）"));
    }

    // 稀疏/密集归一：按弧长等距重采样
    let mut dense = resample(&fitted, RESAMPLE_STEP_M);
    // 打卡点仍远离路径时插入平滑绕行弯
    let detours = insert_detours(&mut dense, &cps);
    let max_cp_residual = cps
        .iter()
        .map(|c| dense.iter().map(|p| dist2(*p, *c)).fold(f64::INFINITY, f64::min).sqrt())
        .fold(0.0f64, f64::max);

    let (cycle, closed) = make_cycle(&dense);
    let arcs = build_arcs(&cycle);
    Ok(PreparedRing {
        ring: (cycle, arcs, cc),
        name: route.name.clone(),
        src_points: src.len(),
        path_len_m: round_to(polyline_len(&dense), 1),
        closed,
        max_cp_residual_m: round_to(max_cp_residual, 1),
        detours,
    })
}

/// 把外部轨迹文件转换为内嵌固定轨迹 JSON 并写出，返回摘要日志。
/// 支持 GPX（trkpt）/ KML（coordinates）/ TCX（LatitudeDegrees）/ JSON。
pub fn convert_route_file(
    input: &str,
    coord: &str,
    name: &str,
    out_path: &str,
) -> Result<Vec<String>, String> {
    let text = std::fs::read_to_string(input).map_err(|e| format!("读取 {input} 失败: {e}"))?;
    let raw = if text.contains("<trkpt") {
        parse_gpx_track(&text)
    } else if text.contains("<coordinates") {
        parse_kml_coordinates(&text)
    } else if text.contains("LatitudeDegrees") {
        parse_tcx_trackpoints(&text)
    } else if text.trim_start().starts_with('{') || text.trim_start().starts_with('[') {
        parse_json_track(&text)?
    } else {
        return Err("无法识别轨迹格式（支持 GPX/KML/TCX/JSON）".into());
    };
    if raw.len() < 8 {
        return Err(format!("轨迹点数不足（{} < 8）", raw.len()));
    }
    if !matches!(coord, "wgs84" | "gcj02" | "bd09") {
        return Err(format!("coordType 无效: {coord}（wgs84/gcj02/bd09）"));
    }
    let mut pts = raw.clone();
    pts.dedup_by(|a, b| dist2(*a, *b) < 1e-10);
    // 度 → 平面米（以首点为原点），后续剔除/平滑/重采样全部在米制空间进行
    let origin = pts[0];
    let to_plane = |p: &(f64, f64)| {
        ((p.1 - origin.1) * MET_PER_DEG_LNG, (p.0 - origin.0) * MET_PER_DEG_LAT)
    };
    let mut plane: Vec<(f64, f64)> = pts.iter().map(&to_plane).collect();
    // 剔除孤立漂移尖峰：与前后两点均相距超过 150m 的点不可信（真实跑步 5s 采样
    // 相邻间距通常 <60m），反复剔除至稳定
    let mut spikes = 0usize;
    loop {
        let mut drop = vec![false; plane.len()];
        for i in 1..plane.len().saturating_sub(1) {
            let back = dist2(plane[i - 1], plane[i]).sqrt();
            let fwd = dist2(plane[i], plane[i + 1]).sqrt();
            if back > 150.0 && fwd > 150.0 {
                drop[i] = true;
            }
        }
        let dropped: usize = drop.iter().filter(|d| **d).count();
        if dropped == 0 {
            break;
        }
        spikes += dropped;
        let mut kept = Vec::with_capacity(plane.len() - dropped);
        for (i, p) in plane.iter().enumerate() {
            if !drop[i] {
                kept.push(*p);
            }
        }
        plane = kept;
        if plane.len() < 8 || spikes > raw.len() / 4 {
            return Err(format!("漂移点过多（剔除 {spikes} 个），轨迹数据质量不足"));
        }
    }
    smooth3(&mut plane);
    let mut warns = Vec::new();
    for (i, w) in plane.windows(2).enumerate() {
        let d = dist2(w[0], w[1]).sqrt();
        if d > 120.0 {
            warns.push(format!("  ⚠ 点 {i} 与下一点相距 {:.0}m（疑似 GPS 漂移）", d));
        }
    }
    let dense_plane = resample(&plane, RESAMPLE_STEP_M);
    // 环线检测：多圈跑（校园小环线等）会在 ~100m 弧长之外多次回到起点附近；
    // 此时截取一圈作为闭环（生成器会按任务距离自动循环），比整条螺旋线往返更真实
    let mut revisits: Vec<(usize, f64)> = Vec::new();
    {
        let mut arc = 0.0f64;
        for i in 1..dense_plane.len() {
            arc += dist2(dense_plane[i - 1], dense_plane[i]).sqrt();
            if arc >= 100.0 {
                let d0 = dist2(dense_plane[i], dense_plane[0]).sqrt();
                if d0 <= 30.0 {
                    revisits.push((i, d0));
                }
            }
        }
    }
    let mut laps_note = String::new();
    let dense_plane = if let Some(&(first_idx, _)) = revisits.first() {
        // 取第一次回到起点的经过段，段内取离起点最近的点作为闭环点
        let mut best = (first_idx, revisits[0].1);
        for &(i, d) in &revisits {
            if i > first_idx + 40 {
                break;
            }
            if d < best.1 {
                best = (i, d);
            }
        }
        let lap_arc: f64 = dense_plane[..=best.0].windows(2).map(|w| dist2(w[0], w[1]).sqrt()).sum();
        // 圈数 = 重访经过段个数 + 1
        let mut passes = 1usize;
        for w in revisits.windows(2) {
            if w[1].0 - w[0].0 > 40 {
                passes += 1;
            }
        }
        laps_note = format!("，检测到环线（约 {} 圈），截取一圈 {:.0}m 作为闭环", passes + 1, lap_arc);
        dense_plane[..=best.0].to_vec()
    } else {
        dense_plane
    };
    let len = polyline_len(&dense_plane);
    let gap = dist2(dense_plane[0], *dense_plane.last().unwrap()).sqrt();
    let closed = gap <= CLOSE_GAP_M;
    let dense: Vec<(f64, f64)> = dense_plane
        .iter()
        .map(|q| to_bd(q.0, q.1, origin.0, origin.1))
        .collect();

    let body: Vec<String> = dense
        .iter()
        .map(|p| format!("[{:.7},{:.7}]", round_to(p.0, 7), round_to(p.1, 7)))
        .collect();
    let doc = format!(
        "{{\n  \"enabled\": true,\n  \"name\": \"{}\",\n  \"coordType\": \"{}\",\n  \"points\": [{}\n  ]\n}}",
        name,
        coord,
        body.chunks(4)
            .map(|c| format!("\n    {}", c.join(", ")))
            .collect::<Vec<_>>()
            .join(",")
    );
    if let Some(parent) = std::path::Path::new(out_path).parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    std::fs::write(out_path, doc).map_err(|e| format!("写出 {out_path} 失败: {e}"))?;
    let mut out = vec![
        format!("√ 已转换 {} → {out_path}", input),
        format!("  原始 {} 点（剔除漂移尖峰 {spikes} 个），重采样后 {} 点（间距 {}m）", raw.len(), dense.len(), RESAMPLE_STEP_M),
        format!("  路径长度 {:.0}m，{}（首尾相距 {:.0}m）{}", len, if closed { "闭环" } else { "开放路径（运行时往返）" }, gap, laps_note),
        format!("  coordType={coord}；请核对首尾漂移警告后重新构建 APK"),
    ];
    out.extend(warns);
    Ok(out)
}

// ---------- 外部轨迹解析 ----------

/// GPX trkpt → (lat, lng)。
pub fn parse_gpx_track(text: &str) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(start) = rest.find("<trkpt") {
        rest = &rest[start..];
        let Some(end) = rest.find('>') else { break };
        let head = &rest[..end];
        let (Some(lat), Some(lon)) = (xml_attr(head, "lat"), xml_attr(head, "lon")) else {
            rest = &rest[end..];
            continue;
        };
        out.push((lat, lon));
        rest = &rest[end..];
    }
    out
}

fn xml_attr(head: &str, name: &str) -> Option<f64> {
    let needle = format!("{name}=");
    let pos = head.find(&needle)?;
    let tail = head[pos + needle.len()..].trim_start();
    let quote = tail.chars().next()?;
    let tail = tail.strip_prefix(quote)?;
    let end = tail.find(quote)?;
    tail[..end].trim().parse().ok()
}

/// KML <coordinates>（lon,lat[,alt] 空白分隔）→ (lat, lng)。
pub fn parse_kml_coordinates(text: &str) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(start) = rest.find("<coordinates") {
        rest = &rest[start..];
        let Some(gt) = rest.find('>') else { break };
        let body = &rest[gt + 1..];
        let Some(close) = body.find("</coordinates>") else { break };
        for token in body[..close].split_whitespace() {
            let mut parts = token.split(',');
            let (Some(lon), Some(lat)) = (parts.next(), parts.next()) else { continue };
            if let (Ok(lon), Ok(lat)) = (lon.trim().parse::<f64>(), lat.trim().parse::<f64>()) {
                out.push((lat, lon));
            }
        }
        rest = &body[close..];
    }
    out
}

/// TCX Trackpoint（LatitudeDegrees/LongitudeDegrees）→ (lat, lng)。
pub fn parse_tcx_trackpoints(text: &str) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(lat_pos) = rest.find("<LatitudeDegrees") {
        rest = &rest[lat_pos..];
        let Some(gt) = rest.find('>') else { break };
        let body = &rest[gt + 1..];
        let Some(lat_end) = body.find("</LatitudeDegrees>") else { break };
        let Ok(lat) = body[..lat_end].trim().parse::<f64>() else {
            rest = &body[lat_end..];
            continue;
        };
        let after = &body[lat_end..];
        let Some(lng_pos) = after.find("<LongitudeDegrees") else { break };
        let after = &after[lng_pos..];
        let Some(lgt) = after.find('>') else { break };
        let lbody = &after[lgt + 1..];
        let Some(lng_end) = lbody.find("</LongitudeDegrees>") else { break };
        let Ok(lng) = lbody[..lng_end].trim().parse::<f64>() else { break };
        out.push((lat, lng));
        rest = &lbody[lng_end..];
    }
    out
}

/// JSON 轨迹：{"points":[[lat,lng],…]}、[[lat,lng],…] 或 [{"lat":..,"lon/lng":..},…]。
/// 兼容 obs-get 导出的历史记录结构：顶层 run_data（gzip 解码后的 JSON 字符串）内含
/// allLocJson（点集 JSON 字符串，点键 gLat/gLng 为 GCJ-02，lat/lng 为 -1 占位）。
pub fn parse_json_track(text: &str) -> Result<Vec<(f64, f64)>, String> {
    let v: Value = serde_json::from_str(text).map_err(|e| format!("JSON 解析失败: {e}"))?;
    let v = unwrap_record_wrapper(v);
    let arr = match &v {
        Value::Array(a) => a.clone(),
        Value::Object(o) => o
            .get("points")
            .and_then(Value::as_array)
            .cloned()
            .ok_or("JSON 轨迹缺少 points 数组")?,
        _ => return Err("JSON 轨迹缺少 points 数组".into()),
    };
    let mut out = Vec::new();
    for item in arr {
        if let Some(pair) = item.as_array() {
            if pair.len() >= 2 {
                if let (Some(lat), Some(lng)) = (pair[0].as_f64(), pair[1].as_f64()) {
                    out.push((lat, lng));
                    continue;
                }
            }
        }
        if let Some(o) = item.as_object() {
            // 真实记录的漂移点（type=-1）位置不可信，剔除
            if o.get("type").and_then(Value::as_i64) == Some(-1) {
                continue;
            }
            // gLat/gLng 优先于 lat/lng（记录点 lat/lng 为 -1 占位）
            let lat = coord_of(o, &["gLat", "lat", "latitude"], -90.0..=90.0);
            let lng = coord_of(o, &["gLng", "lon", "lng", "longitude"], -180.0..=180.0);
            if let (Some(lat), Some(lng)) = (lat, lng) {
                out.push((lat, lng));
            }
        }
    }
    Ok(out)
}

/// 逐层解开历史记录包装：run_data / allLocJson 都可能是 JSON 字符串字段。
fn unwrap_record_wrapper(v: Value) -> Value {
    const KEYS: [&str; 2] = ["run_data", "allLocJson"];
    let mut cur = v;
    for _ in 0..3 {
        let Value::Object(map) = &cur else { break };
        let inner = KEYS.iter().find_map(|k| {
            let raw = map.get(*k)?;
            match raw {
                Value::String(s) if s.trim_start().starts_with('{') || s.trim_start().starts_with('[') => {
                    serde_json::from_str::<Value>(s).ok()
                }
                Value::Object(_) | Value::Array(_) => Some(raw.clone()),
                _ => None,
            }
        });
        match inner {
            Some(next) => cur = next,
            None => break,
        }
    }
    cur
}

/// 取第一个存在、非 -1 占位且落在合理范围的坐标值。
fn coord_of(o: &serde_json::Map<String, Value>, keys: &[&str], range: std::ops::RangeInclusive<f64>) -> Option<f64> {
    for k in keys {
        if let Some(v) = o.get(*k).and_then(Value::as_f64) {
            if v != -1.0 && range.contains(&v) {
                return Some(v);
            }
        }
    }
    None
}

// ---------- 坐标系 ----------

/// WGS-84 → GCJ-02（国标偏移；境外原样返回）。返回 (lat, lng)。
pub fn wgs84_to_gcj02(lat: f64, lng: f64) -> (f64, f64) {
    const A: f64 = 6_378_245.0;
    const EE: f64 = 0.006_693_421_622_965_943_23;
    if lng < 72.004 || lng > 137.8347 || lat < 0.8293 || lat > 55.8271 {
        return (lat, lng);
    }
    let dlat = transform_lat(lng - 105.0, lat - 35.0);
    let dlng = transform_lng(lng - 105.0, lat - 35.0);
    let radlat = lat / 180.0 * std::f64::consts::PI;
    let mut magic = radlat.sin();
    magic = 1.0 - EE * magic * magic;
    let sqrtmagic = magic.sqrt();
    let dlat = (dlat * 180.0) / ((A * (1.0 - EE)) / (magic * magic) * std::f64::consts::PI);
    let dlng = (dlng * 180.0) / (A / sqrtmagic * radlat.cos() * std::f64::consts::PI);
    (lat + dlat, lng + dlng)
}

fn transform_lat(x: f64, y: f64) -> f64 {
    let mut ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * x.abs().sqrt();
    ret += (20.0 * (6.0 * x * std::f64::consts::PI).sin() + 20.0 * (2.0 * x * std::f64::consts::PI).sin()) * 2.0 / 3.0;
    ret += (20.0 * (y * std::f64::consts::PI).sin() + 40.0 * (y / 3.0 * std::f64::consts::PI).sin()) * 2.0 / 3.0;
    ret += (160.0 * (y / 12.0 * std::f64::consts::PI).sin() + 320.0 * (y * std::f64::consts::PI / 30.0).sin()) * 2.0 / 3.0;
    ret
}

fn transform_lng(x: f64, y: f64) -> f64 {
    let mut ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * x.abs().sqrt();
    ret += (20.0 * (6.0 * x * std::f64::consts::PI).sin() + 20.0 * (2.0 * x * std::f64::consts::PI).sin()) * 2.0 / 3.0;
    ret += (20.0 * (x * std::f64::consts::PI).sin() + 40.0 * (x / 3.0 * std::f64::consts::PI).sin()) * 2.0 / 3.0;
    ret += (150.0 * (x / 12.0 * std::f64::consts::PI).sin() + 300.0 * (x / 30.0 * std::f64::consts::PI).sin()) * 2.0 / 3.0;
    ret
}

/// GCJ-02 → BD-09。返回 (lat, lng)。
pub fn gcj02_to_bd09(lat: f64, lng: f64) -> (f64, f64) {
    let x_pi = std::f64::consts::PI * 3000.0 / 180.0;
    let z = (lng * lng + lat * lat).sqrt() + 0.00002 * (lat * x_pi).sin();
    let theta = lat.atan2(lng) + 0.000003 * (lng * x_pi).cos();
    (z * theta.sin() + 0.006, z * theta.cos() + 0.0065)
}

/// 按声明坐标系归一到 BD-09。
pub fn to_bd09(points: &[(f64, f64)], coord_type: &str) -> Result<Vec<(f64, f64)>, String> {
    match coord_type {
        "bd09" => Ok(points.to_vec()),
        "gcj02" => Ok(points.iter().map(|p| gcj02_to_bd09(p.0, p.1)).collect()),
        "wgs84" => Ok(points
            .iter()
            .map(|p| {
                let gcj = wgs84_to_gcj02(p.0, p.1);
                gcj02_to_bd09(gcj.0, gcj.1)
            })
            .collect()),
        other => Err(format!("未知 coordType: {other}")),
    }
}

// ---------- 几何 ----------

fn centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len().max(1) as f64;
    (
        pts.iter().map(|p| p.0).sum::<f64>() / n,
        pts.iter().map(|p| p.1).sum::<f64>() / n,
    )
}

/// Weiszfeld 迭代求几何中位点：对离群打卡点稳健（质心会被个别远点拉偏，
/// 中位点只被拉动 ~离群距离/点数），对对称分布无偏。
fn robust_center(pts: &[(f64, f64)]) -> (f64, f64) {
    let mut c = centroid(pts);
    for _ in 0..32 {
        let (mut num_x, mut num_y, mut den) = (0.0f64, 0.0f64, 0.0f64);
        for p in pts {
            let d = dist2(*p, c).sqrt().max(1e-3);
            num_x += p.0 / d;
            num_y += p.1 / d;
            den += 1.0 / d;
        }
        if den <= 0.0 {
            break;
        }
        let next = (num_x / den, num_y / den);
        if dist2(next, c) < 1e-9 {
            c = next;
            break;
        }
        c = next;
    }
    c
}

fn dist2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

fn polyline_len(pts: &[(f64, f64)]) -> f64 {
    pts.windows(2).map(|w| dist2(w[0], w[1]).sqrt()).sum()
}

/// 刚体贴合搜索：返回 (旋转弧度, 等比缩放, 平移)。p' = 平移 + k·R(θ)·(p − 路径质心)。
/// 代价 =（超过吸附半径的打卡点数）×1e7 + Σ min(最近距离², cap)：
/// 先保证尽量多的打卡点落在吸附半径内（决定绕行次数），再压低整体残差；
/// 逐点截断使远离路径的打卡点不拖偏整体贴合。三阶段网格：粗搜 → 细化 → 全分辨率平移抛光。
fn fit_transform(path: &[(f64, f64)], cps: &[(f64, f64)]) -> (f64, f64, (f64, f64)) {
    let pc = centroid(path);
    let cap = (SNAP_RADIUS_M * 1.2).powi(2);
    let snap2 = SNAP_RADIUS_M * SNAP_RADIUS_M;
    let cost = |theta: f64, k: f64, t: (f64, f64), sample: &[(f64, f64)]| -> f64 {
        let (st, ct) = theta.sin_cos();
        let mut total = 0.0f64;
        let mut over = 0usize;
        for c in cps {
            let mut best = f64::INFINITY;
            for p in sample {
                let dx = k * (ct * (p.0 - pc.0) - st * (p.1 - pc.1)) + t.0 - c.0;
                let dy = k * (st * (p.0 - pc.0) + ct * (p.1 - pc.1)) + t.1 - c.1;
                let d2 = dx * dx + dy * dy;
                if d2 < best {
                    best = d2;
                }
            }
            if best > snap2 {
                over += 1;
            }
            total += best.min(cap);
        }
        over as f64 * 1.0e7 + total
    };
    let mk_sample = |target: usize| -> Vec<(f64, f64)> {
        let stride = ((path.len() + target - 1) / target).max(1);
        path.iter().step_by(stride).copied().collect()
    };
    let s64 = mk_sample(64);
    let s256 = mk_sample(256);
    let sfull = mk_sample(1024);
    // 平移范围按几何定：锚点（中位点/质心）与真实中心的偏差可达
    // max|cp| + 路径半径 量级（打卡点稀疏不对称时中位点明显偏离路径中心）
    let cp_reach = cps.iter().map(|c| c.0.hypot(c.1)).fold(0.0f64, f64::max);
    let q_reach = s64.iter().map(|p| p.0.hypot(p.1)).fold(0.0f64, f64::max);
    let coarse_range = (cp_reach + q_reach).max(100.0);
    let step1 = (coarse_range / 10.0).ceil().max(10.0);

    // 阶段1：粗搜 θ(10°) × k(6) × 平移(±range1 步 step1)
    let mut best = (0.0f64, 1.0f64, (0.0f64, 0.0f64), f64::INFINITY);
    for i in 0..36 {
        let theta = i as f64 * std::f64::consts::TAU / 36.0;
        for k in [0.5f64, 0.75, 1.0, 1.25, 1.5, 2.0] {
            for a in -10i32..=10 {
                for b in -10i32..=10 {
                    let t = (a as f64 * step1, b as f64 * step1);
                    let c = cost(theta, k, t, &s64);
                    if c < best.3 {
                        best = (theta, k, t, c);
                    }
                }
            }
        }
    }
    // 阶段2：细化 θ(±8° 步 1°) × k(±0.24 步 0.04) × 平移(±2·step1 步 step1/4)
    let (t0, k0, t0v, _) = best;
    let step2 = (step1 / 4.0).max(1.0);
    best = (t0, k0, t0v, cost(t0, k0, t0v, &s256));
    for i in -8i32..=8 {
        for j in -6i32..=6 {
            let theta = t0 + i as f64 * 1.0f64.to_radians();
            let k = (k0 + j as f64 * 0.04).max(0.2);
            for a in -8i32..=8 {
                for b in -8i32..=8 {
                    let t = (t0v.0 + a as f64 * step2, t0v.1 + b as f64 * step2);
                    let c = cost(theta, k, t, &s256);
                    if c < best.3 {
                        best = (theta, k, t, c);
                    }
                }
            }
        }
    }
    // 阶段3：全分辨率平移抛光（±10m 步 2.5m）
    let (t0, k0, t0v, _) = best;
    best = (t0, k0, t0v, cost(t0, k0, t0v, &sfull));
    for a in -4i32..=4 {
        for b in -4i32..=4 {
            let t = (t0v.0 + a as f64 * 2.5, t0v.1 + b as f64 * 2.5);
            let c = cost(t0, k0, t, &sfull);
            if c < best.3 {
                best = (t0, k0, t, c);
            }
        }
    }
    (best.0, best.1, best.2)
}

/// 按弧长等距重采样（保留首尾点），同时消除原地驻留产生的密集重复点。
fn resample(pts: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0]];
    let mut carry = 0.0f64;
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let seg = dist2(a, b).sqrt();
        if seg < 1e-9 {
            continue;
        }
        let mut d = step_m - carry;
        while d < seg {
            let t = d / seg;
            out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
            d += step_m;
        }
        carry = seg - (d - step_m);
    }
    let last = *pts.last().unwrap();
    if dist2(*out.last().unwrap(), last) > 1e-9 {
        out.push(last);
    }
    out
}

/// 三点滑动平均（首尾保留原值），抑制 GPS 毛刺。
fn smooth3(pts: &mut Vec<(f64, f64)>) {
    if pts.len() < 3 {
        return;
    }
    let src = pts.clone();
    for i in 1..src.len() - 1 {
        pts[i] = (
            (src[i - 1].0 + src[i].0 + src[i + 1].0) / 3.0,
            (src[i - 1].1 + src[i].1 + src[i + 1].1) / 3.0,
        );
    }
}

/// 打卡点距路径超过 SNAP_RADIUS_M 时插入平滑绕行弯（二次贝塞尔，精确过打卡点），返回次数。
fn insert_detours(dense: &mut Vec<(f64, f64)>, cps: &[(f64, f64)]) -> usize {
    let mut count = 0;
    for cp in cps {
        let Some((bi, d0)) = dense
            .iter()
            .enumerate()
            .map(|(i, p)| (i, dist2(*p, *cp)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(i, d)| (i, d.sqrt()))
        else {
            continue;
        };
        if d0 <= SNAP_RADIUS_M {
            continue;
        }        let mut a = bi;
        let mut acc = 0.0;
        while a > 0 && acc < DETOUR_SHOULDER_M {
            acc += dist2(dense[a], dense[a - 1]).sqrt();
            a -= 1;
        }
        let mut b = bi;
        let mut acc = 0.0;
        while b + 1 < dense.len() && acc < DETOUR_SHOULDER_M {
            acc += dist2(dense[b], dense[b + 1]).sqrt();
            b += 1;
        }
        if b - a < 2 {
            continue;
        }
        let (a_pt, b_pt) = (dense[a], dense[b]);
        let q = (
            2.0 * cp.0 - 0.5 * (a_pt.0 + b_pt.0),
            2.0 * cp.1 - 0.5 * (a_pt.1 + b_pt.1),
        );
        let steps = 32usize;
        let bend: Vec<(f64, f64)> = (1..steps)
            .map(|s| {
                let t = s as f64 / steps as f64;
                let u = 1.0 - t;
                (
                    u * u * a_pt.0 + 2.0 * u * t * q.0 + t * t * b_pt.0,
                    u * u * a_pt.1 + 2.0 * u * t * q.1 + t * t * b_pt.1,
                )
            })
            .collect();
        dense.splice(a + 1..b, bend);
        count += 1;
    }
    count
}

/// 闭环（首尾相近，补一段回程渐变）或往返（正放 + 逆放拼成可循环路径）。
fn make_cycle(dense: &[(f64, f64)]) -> (Vec<(f64, f64)>, bool) {
    let n = dense.len();
    let gap = dist2(dense[0], dense[n - 1]).sqrt();
    if gap <= CLOSE_GAP_M {
        let mut out = dense.to_vec();
        if gap > 6.0 {
            let steps = (gap / 6.0).ceil() as usize;
            for s in 1..steps {
                let t = s as f64 / steps as f64;
                out.push((
                    dense[n - 1].0 + (dense[0].0 - dense[n - 1].0) * t,
                    dense[n - 1].1 + (dense[0].1 - dense[n - 1].1) * t,
                ));
            }
        }
        (out, true)
    } else {
        let mut out = dense.to_vec();
        for i in (1..n - 1).rev() {
            out.push(dense[i]);
        }
        (out, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::geom::{ring_point_at, to_bd};
    use crate::track::wire::bd09_to_gcj02;

    #[test]
    fn datum_conversions_round_trip_and_offsets() {
        // gcj→bd 与 bd→gcj 互逆
        let (lat, lng) = (38.901678, 121.540241);
        let bd = gcj02_to_bd09(lat, lng);
        let back = bd09_to_gcj02(bd.0, bd.1);
        assert!((back.0 - lat).abs() < 2e-6, "lat={}", back.0);
        assert!((back.1 - lng).abs() < 2e-6, "lng={}", back.1);
        // 中国境内 wgs→gcj 偏移量在合理区间
        let gcj = wgs84_to_gcj02(38.901678, 121.540241);
        let d = ((gcj.0 - 38.901678) * MET_PER_DEG_LAT).hypot((gcj.1 - 121.540241) * MET_PER_DEG_LNG);
        assert!((100.0..900.0).contains(&d), "偏移 {d}m");
        // 境外原样
        let out = wgs84_to_gcj02(35.0, 139.0);
        assert_eq!(out, (35.0, 139.0));
    }

    #[test]
    fn resample_uniform_and_dedupes() {
        // 稀疏折线：100m 间距 → 3m 重采样
        let pts: Vec<(f64, f64)> = (0..10).map(|i| (i as f64 * 100.0, 0.0)).collect();
        let dense = resample(&pts, 3.0);
        assert!(dense.len() > 300, "n={}", dense.len());
        let denses = resample(&[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)], 3.0);
        assert_eq!(denses.len(), 2);
        // 首尾保留
        assert_eq!(dense[0], (0.0, 0.0));
        assert_eq!(*dense.last().unwrap(), (900.0, 0.0));
    }

    #[test]
    fn parse_gpx_kml_tcx() {
        let gpx = r#"<gpx><trk><trkseg>
            <trkpt lat="38.9" lon="121.54"></trkpt>
            <trkpt lat="38.901" lon="121.541"/><trkpt lat="38.902" lon="121.542"></trkpt>
        </trkseg></trk></gpx>"#;
        assert_eq!(parse_gpx_track(gpx).len(), 3);
        let kml = r#"<kml><LineString><coordinates>121.54,38.9,0 121.541,38.901 121.542,38.902,3</coordinates></LineString></kml>"#;
        let kpts = parse_kml_coordinates(kml);
        assert_eq!(kpts.len(), 3);
        assert_eq!(kpts[0], (38.9, 121.54));
        let tcx = r#"<t><Trackpoint><LatitudeDegrees>38.9</LatitudeDegrees><LongitudeDegrees>121.54</LongitudeDegrees></Trackpoint><Trackpoint><LatitudeDegrees>38.901</LatitudeDegrees><LongitudeDegrees>121.541</LongitudeDegrees></Trackpoint></t>"#;
        assert_eq!(parse_tcx_trackpoints(tcx).len(), 2);
    }

    #[test]
    fn route_json_validation() {
        assert!(parse_route_json(b"{\"enabled\":false,\"points\":[]}").unwrap().is_none());
        assert!(parse_route_json(b"{\"enabled\":true,\"coordType\":\"wgs84\",\"points\":[[1,2]]}").is_err());
        assert!(parse_route_json(b"not json").is_err());
        let pts: Vec<String> = (0..10).map(|i| format!("[38.9{i},121.54]")).collect();
        let doc = format!("{{\"enabled\":true,\"name\":\"t\",\"coordType\":\"bd09\",\"points\":[{}]}}", pts.join(","));
        let r = parse_route_json(doc.as_bytes()).unwrap().unwrap();
        assert_eq!(r.name, "t");
        assert_eq!(r.points.len(), 10);
    }

    fn circle_route_bd(center: (f64, f64), radius_m: f64, rot_deg: f64, wiggle: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let ang = rot_deg.to_radians() + i as f64 / n as f64 * std::f64::consts::TAU;
                let r = radius_m + wiggle * (6.0 * ang).sin();
                let x = r * ang.cos();
                let y = r * ang.sin();
                to_bd(x, y, center.0, center.1)
            })
            .collect()
    }

    /// 校内测试打卡点（BD 系，与 track::mod 测试同一片区域量级）。
    fn test_checkpoints(center: (f64, f64), radius_m: f64, rot_deg: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let ang = rot_deg.to_radians() + i as f64 / n as f64 * std::f64::consts::TAU;
                to_bd(radius_m * ang.cos(), radius_m * ang.sin(), center.0, center.1)
            })
            .collect()
    }

    #[test]
    fn closed_loop_fits_checkpoints_and_builds_track() {
        let center = (38.901, 121.541);
        // 真实路线：旋转 33° 的带抖动环，半径 280m
        let route = FixedRoute {
            name: "测试环".into(),
            coord_type: "bd09".into(),
            points: circle_route_bd(center, 280.0, 33.0, 15.0, 360),
        };
        // 打卡点在同一圆周上（旋转 17°）
        let cps = test_checkpoints(center, 280.0, 17.0, 5);
        let prepared = prepare_ring(&cps, &route).unwrap();
        assert!(prepared.closed, "应为闭环");
        assert_eq!(prepared.detours, 0, "不应绕行");
        assert!(prepared.max_cp_residual_m < 20.0, "残差 {}", prepared.max_cp_residual_m);
        assert!((prepared.path_len_m - 1759.0).abs() < 80.0, "路径长 {}", prepared.path_len_m);
        // 生成轨迹：总距精确、打卡点吸附 <1m、正常点贴近路径
        let track = crate::track::generator::build_on_ring(
            3300.0, 1220, 42, 1_788_958_186_123, &cps, prepared.ring.clone(),
        );
        let (dense, arcs, pc) = &prepared.ring;
        assert!((track.totalDistance - 3300.0).abs() < 0.5, "dist={}", track.totalDistance);
        for cp in &cps {
            let min_m = track
                .locations
                .iter()
                .map(|p| (((p.gLat - cp.0) * MET_PER_DEG_LAT).powi(2) + ((p.gLng - cp.1) * MET_PER_DEG_LNG).powi(2)).sqrt())
                .fold(f64::INFINITY, f64::min);
            assert!(min_m < 1.0, "打卡点落位 {min_m}m");
        }
        // 形状保持：正常点（type≠-1）到路径的距离中位数 < 6m
        let mut devs: Vec<f64> = track
            .locations
            .iter()
            .filter(|p| p.ptype != -1)
            .map(|p| {
                let x = (p.gLng - pc.1) * MET_PER_DEG_LNG;
                let y = (p.gLat - pc.0) * MET_PER_DEG_LAT;
                (0..256)
                    .map(|s| {
                        let (dx, dy) = ring_point_at(dense, arcs, s as f64 / 256.0 * arcs.last().unwrap());
                        ((dx - x).powi(2) + (dy - y).powi(2)).sqrt()
                    })
                    .fold(f64::INFINITY, f64::min)
            })
            .collect();
        devs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = devs[devs.len() / 2];
        assert!(median < 6.0, "中位偏差 {median}m");
    }

    #[test]
    fn far_checkpoint_triggers_detour() {
        let center = (38.901, 121.541);
        let route = FixedRoute {
            name: "测试环".into(),
            coord_type: "bd09".into(),
            points: circle_route_bd(center, 280.0, 0.0, 8.0, 300),
        };
        let mut cps = test_checkpoints(center, 280.0, 0.0, 5);
        // 把一个打卡点挪出路径 100m
        cps[2] = to_bd(380.0, 0.0, center.0, center.1);
        let prepared = prepare_ring(&cps, &route).unwrap();
        assert_eq!(prepared.detours, 1, "应有 1 处绕行");
        // 未绕行的打卡点残差须在吸附半径内（生成器吸附后精确落位）
        assert!(prepared.max_cp_residual_m < 30.0, "残差 {}", prepared.max_cp_residual_m);
    }

    #[test]
    fn open_path_becomes_round_trip() {
        let center = (38.901, 121.541);
        // 开放路径：3/4 圆弧
        let pts: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let ang = i as f64 / 200.0 * 1.5 * std::f64::consts::TAU;
                to_bd(280.0 * ang.cos(), 280.0 * ang.sin(), center.0, center.1)
            })
            .collect();
        let route = FixedRoute { name: "弧".into(), coord_type: "bd09".into(), points: pts };
        let cps = test_checkpoints(center, 280.0, 0.0, 4);
        let prepared = prepare_ring(&cps, &route).unwrap();
        assert!(!prepared.closed, "开放路径应往返");
        let track = crate::track::generator::build_on_ring(
            2200.0, 900, 7, 1_788_958_186_123, &cps, prepared.ring,
        );
        assert!((track.totalDistance - 2200.0).abs() < 2.0);
        // 往返拼接处连续：正常点相邻位移不超过速度上限 × 间隔的合理倍数
        for w in track.locations.windows(2) {
            if w[0].ptype == -1 || w[1].ptype == -1 {
                continue;
            }
            let dt = (w[1].totalTime - w[0].totalTime).max(1) as f64;
            let d = (((w[1].gLat - w[0].gLat) * MET_PER_DEG_LAT).powi(2)
                + ((w[1].gLng - w[0].gLng) * MET_PER_DEG_LNG).powi(2))
            .sqrt();
            assert!(d < SPEED_CAP * dt * 1.6 + 12.0, "跳变 {d}m @ {dt}s");
        }
    }

    const SPEED_CAP: f64 = 6.3;

    #[test]
    fn parse_json_track_supports_record_dump() {
        // 模拟 obs-get 导出的历史记录：run_data/allLocJson 均为字符串字段，
        // 点键 gLat/gLng 为 GCJ，lat/lng 为 -1 占位，含 type=-1 漂移点
        let pts = serde_json::json!([
            {"gLat": 38.901, "gLng": 121.541, "lat": -1, "lng": -1, "type": 0},
            {"gLat": -1, "gLng": -1, "lat": -1, "lng": -1, "type": -1},
            {"gLat": 38.9015, "gLng": 121.5415, "lat": -1, "lng": -1, "type": 0}
        ])
        .to_string();
        let wrapper = serde_json::json!({"useZip": false, "allLocJson": pts}).to_string();
        let doc = serde_json::json!({"rrid": "1320403809", "uuid": "ABC", "run_data": wrapper}).to_string();
        let parsed = parse_json_track(&doc).unwrap();
        assert_eq!(parsed.len(), 2, "漂移点应被剔除: {parsed:?}");
        assert_eq!(parsed[0], (38.901, 121.541));
    }

    #[test]
    fn convert_drops_isolated_spike() {
        let dir = std::env::temp_dir().join("neko-fixed-spike");
        let _ = std::fs::create_dir_all(&dir);
        let gpx_path = dir.join("spike.gpx");
        let out_path = dir.join("spike.json");
        // 一条 100m 直线上第 5 点漂移 400m 外
        let mut pts: Vec<(f64, f64)> = (0..40).map(|i| (0.0, i as f64 * 3.0)).collect();
        pts.insert(20, (400.0, 60.0));
        let gpx: String = pts
            .iter()
            .map(|p| format!("<trkpt lat=\"{:.7}\" lon=\"{:.7}\"/>", 38.9 + p.0 / MET_PER_DEG_LAT, 121.5 + p.1 / MET_PER_DEG_LNG))
            .collect();
        std::fs::write(&gpx_path, format!("<gpx>{gpx}</gpx>")).unwrap();
        let lines = convert_route_file(gpx_path.to_str().unwrap(), "wgs84", "spike", out_path.to_str().unwrap()).unwrap();
        assert!(lines.iter().any(|l| l.contains("剔除漂移尖峰 1 个")), "{lines:?}");
        let doc = std::fs::read_to_string(&out_path).unwrap();
        // 漂移点被剔除后路径长度接近 117m 而非来回 800m+
        assert!(doc.contains("\"points\""));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn convert_extracts_single_lap_from_multi_lap_run() {
        let dir = std::env::temp_dir().join("neko-fixed-lap");
        let _ = std::fs::create_dir_all(&dir);
        let gpx_path = dir.join("laps.gpx");
        let out_path = dir.join("laps.json");
        // 2.4 圈 280m 半径圆（第 3 圈中途停止）：起点在角度 0，最终停在角度 144°
        let n_per_lap = 100;
        let total = (n_per_lap as f64 * 2.4) as usize;
        let pts: Vec<(f64, f64)> = (0..total)
            .map(|i| {
                let ang = i as f64 / n_per_lap as f64 * std::f64::consts::TAU;
                to_bd(280.0 * ang.cos(), 280.0 * ang.sin(), 38.901, 121.541)
            })
            .collect();
        let gpx: String = pts
            .iter()
            .map(|p| format!("<trkpt lat=\"{:.7}\" lon=\"{:.7}\"/>", p.0, p.1))
            .collect();
        std::fs::write(&gpx_path, format!("<gpx>{gpx}</gpx>")).unwrap();
        let lines = convert_route_file(gpx_path.to_str().unwrap(), "bd09", "laps", out_path.to_str().unwrap()).unwrap();
        assert!(lines.iter().any(|l| l.contains("检测到环线")), "{lines:?}");
        let route = parse_route_json(std::fs::read_to_string(&out_path).unwrap().as_bytes()).unwrap().unwrap();
        // 截取后应为一圈：长度约 2π·280 ≈ 1759m
        let src = to_bd09(&route.points, "bd09").unwrap();
        let rc = centroid(&src);
        let plane: Vec<(f64, f64)> = src.iter().map(|p| ((p.1 - rc.1) * MET_PER_DEG_LNG, (p.0 - rc.0) * MET_PER_DEG_LAT)).collect();
        let len = polyline_len(&plane);
        assert!((len - 1759.0).abs() < 120.0, "单圈长 {len}m");
        assert!((dist2(plane[0], *plane.last().unwrap()).sqrt()) < 30.0, "首尾应闭合");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn convert_route_file_end_to_end() {
        let dir = std::env::temp_dir().join("neko-fixed-test");
        let _ = std::fs::create_dir_all(&dir);
        let gpx_path = dir.join("route.gpx");
        let out_path = dir.join("fixed_route.json");
        let center = (38.901, 121.541);
        let pts = circle_route_bd(center, 280.0, 0.0, 0.0, 120);
        let gpx: String = pts
            .iter()
            .map(|p| format!("<trkpt lat=\"{:.7}\" lon=\"{:.7}\"/>", p.0, p.1))
            .collect();
        std::fs::write(&gpx_path, format!("<gpx>{gpx}</gpx>")).unwrap();
        let lines = convert_route_file(
            gpx_path.to_str().unwrap(),
            "bd09",
            "端到端",
            out_path.to_str().unwrap(),
        )
        .unwrap();
        assert!(lines.iter().any(|l| l.contains("闭环")));
        let doc = std::fs::read_to_string(&out_path).unwrap();
        #[cfg(test)]
        eprintln!("[diag3] doc_len={} head={:?} tail={:?}", doc.len(), &doc[..doc.len().min(200)], &doc[doc.len().saturating_sub(120)..]);
        let route = parse_route_json(doc.as_bytes()).unwrap().unwrap();
        assert_eq!(route.name, "端到端");
        assert!(route.points.len() > 100);
        // 转出的 JSON 直接可被 prepare_ring 消费
        let cps = test_checkpoints(center, 280.0, 0.0, 5);
        assert!(prepare_ring(&cps, &route).is_ok());
        let _ = std::fs::remove_dir_all(&dir);
    }
}

