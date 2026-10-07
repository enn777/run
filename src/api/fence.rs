//! 电子围栏:POST /api/v1/getGeoFenceForRun(借鉴上游 v0.3.0 的 fence 接口)。
//!
//! 返回 data.geoFences[],每个围栏含 id/name 与 points[](坐标与打卡点同系:
//! BD-09 lat/lon 与 GCJ glat/glon 可能一方被置 0)。保留原始 JSON 供提交与
//! OBS 的 geoFencesJson 使用——真实 App 每次任务都会上报当前围栏。

use super::client::{get_field, ApiClient};
use serde_json::{json, Value};

pub const FENCE_PATH: &str = "/api/v1/getGeoFenceForRun";

/// 拉取围栏原始数组(逐字段保留 id/name/distance/points,剔除无有效顶点的条目)。
pub fn fetch_geo_fences_raw(
    client: &mut ApiClient,
    log: &mut dyn FnMut(&str),
) -> Result<Value, String> {
    let body = json!({ "geoFenceUpdateTime": 0 }).to_string();
    let biz = client.call("POST", FENCE_PATH, &body, &[])?;
    let fences = get_field(&biz, "geoFences")
        .and_then(|v| v.as_array())
        .cloned()
        .unwrap_or_default();
    let kept: Vec<Value> = fences
        .into_iter()
        .filter(|f| {
            f.get("points")
                .and_then(|v| v.as_array())
                .map(|pts| {
                    pts.iter().any(|p| {
                        let a = p
                            .get("lat")
                            .or_else(|| p.get("glat"))
                            .and_then(Value::as_f64)
                            .unwrap_or(0.0);
                        let o = p
                            .get("lon")
                            .or_else(|| p.get("glon"))
                            .and_then(Value::as_f64)
                            .unwrap_or(0.0);
                        !(a == 0.0 && o == 0.0)
                    })
                })
                .unwrap_or(false)
        })
        .collect();
    log(&format!(
        "[fence] 围栏接口返回 {} 个,有效 {} 个",
        get_field(&biz, "geoFences").and_then(|v| v.as_array()).map(|a| a.len()).unwrap_or(0),
        kept.len()
    ));
    Ok(Value::Array(kept))
}
