//! Parser module for DJI goggles telemetry CSV files.
//!
//! These are telemetry CSVs recorded on DJI goggles, one per session, named
//! `telemetry_YYYYMMDD_HHMMSS_mmm.csv` (seen with a Goggles Integra and an O4
//! Air Unit). Columns are dotted paths: `flight.*` holds the DJI flight
//! controller's OSD and home state, `battery.*` and `camera.*` the smart battery
//! and camera, `link.*` the video link, and `elrs.*` the CRSF telemetry an
//! ExpressLRS link passes through. A DJI drone fills the `flight.osd.*` columns;
//! an FPV quad flying Betaflight/INAV leaves them empty and only has `elrs.*`,
//! so each value prefers the DJI field and falls back to the ELRS one.
//!
//! A row is written every ~200 ms whether or not new telemetry arrived, so ELRS
//! values repeat until the next CRSF frame and are blank until the first one.
//! Rows without a GPS fix are skipped, and so is everything before the air unit
//! first links up: until then the ELRS columns can still hold a stale fix from
//! an earlier session.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use chrono::{DateTime, Utc};
use std::collections::HashMap;

use dji_log_parser::record::osd::FlightMode;

use crate::database::Database;
use crate::dronelogbook_parser::DroneLogbookParser;
use crate::models::{FlightMetadata, TelemetryPoint};
use crate::parser::{LogParser, ParseResult, ParserError};

/// Column name -> index
struct ColumnMap {
    indices: HashMap<String, usize>,
}

impl ColumnMap {
    fn new(headers: &[String]) -> Self {
        let indices = headers
            .iter()
            .enumerate()
            .map(|(i, h)| (h.trim().to_string(), i))
            .collect();
        Self { indices }
    }

    fn get_str<'r>(&self, row: &'r [String], field: &str) -> Option<&'r str> {
        let val = row.get(*self.indices.get(field)?)?.trim();
        if val.is_empty() { None } else { Some(val) }
    }

    /// Finite float, or None for a blank, unparsable, NaN or infinite cell
    fn get_f64(&self, row: &[String], field: &str) -> Option<f64> {
        self.get_str(row, field)?.parse::<f64>().ok().filter(|v| v.is_finite())
    }

    /// First column of `fields` that has a value
    fn first_f64(&self, row: &[String], fields: &[&str]) -> Option<f64> {
        fields.iter().find_map(|f| self.get_f64(row, f))
    }
}

/// A usable fix: both coordinates present, in range and not the 0/0 "no fix" placeholder
fn fix(lat: Option<f64>, lon: Option<f64>) -> Option<(f64, f64)> {
    match (lat, lon) {
        (Some(lat), Some(lon))
            if lat.abs() <= 90.0 && lon.abs() <= 180.0 && (lat.abs() > 1e-6 || lon.abs() > 1e-6) =>
        {
            Some((lat, lon))
        }
        _ => None,
    }
}

/// DJI goggles telemetry CSV parser
pub struct GogglesParser<'a> {
    db: &'a Database,
}

impl<'a> GogglesParser<'a> {
    pub fn new(db: &'a Database) -> Self {
        Self { db }
    }

    /// Check if a file is a DJI goggles telemetry CSV
    pub fn is_goggles_csv(path: &Path) -> bool {
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
        if !ext.eq_ignore_ascii_case("csv") {
            return false;
        }

        if let Ok(file) = File::open(path) {
            let reader = BufReader::new(file);
            if let Some(Ok(first_line)) = reader.lines().next() {
                let headers: Vec<&str> = first_line.split(',').map(|h| h.trim()).collect();
                let has = |name: &str| headers.contains(&name);
                return headers.first() == Some(&"timestamp")
                    && has("link.goggles.name")
                    && ((has("flight.osd.lat_deg") && has("flight.osd.lon_deg"))
                        || (has("elrs.gps.lat_deg") && has("elrs.gps.lon_deg")));
            }
        }
        false
    }

    /// Parse a DJI goggles telemetry CSV file
    pub fn parse(&self, file_path: &Path, file_hash: &str) -> Result<ParseResult, ParserError> {
        let parse_start = std::time::Instant::now();
        log::info!("Parsing DJI goggles telemetry CSV: {:?}", file_path);

        let file = File::open(file_path)?;
        let mut lines = BufReader::new(file).lines();

        let header_line = lines
            .next()
            .ok_or_else(|| ParserError::Parse("Empty CSV file".to_string()))?
            .map_err(|e| ParserError::Parse(format!("Failed to read header: {}", e)))?;
        let headers = DroneLogbookParser::parse_csv_line(&header_line);
        let col_map = ColumnMap::new(&headers);

        let mut points = Vec::new();
        let mut start_time: Option<DateTime<Utc>> = None;
        let mut flight_mode: Option<String> = None;
        let mut drone_model: Option<String> = None;
        let mut home: Option<(f64, f64)> = None;
        // Files without the column can't tell, so they count as linked from the start
        let mut linked = !col_map.indices.contains_key("link.aircraft.name");

        for line_result in lines {
            let line = match line_result {
                Ok(l) => l,
                Err(e) => {
                    log::warn!("Skipping malformed line: {}", e);
                    continue;
                }
            };
            if line.trim().is_empty() {
                continue;
            }
            let row = DroneLogbookParser::parse_csv_line(&line);

            if let Some(name) = col_map.get_str(&row, "link.aircraft.name") {
                // A file that never links keeps its points, the same as one without the column
                if !linked {
                    linked = true;
                    points.clear();
                    start_time = None;
                    home = None;
                    flight_mode = None;
                }
                if drone_model.is_none() {
                    drone_model = Some(if name.starts_with("DJI") { name.to_string() } else { format!("DJI {}", name) });
                }
            }
            // The flight mode is only filled in on some rows; carry it forward
            // so the blanks in between don't read as mode changes. DJI's flyc_state
            // is named the same way as in DJI .txt logs.
            if let Some(mode) = col_map
                .get_f64(&row, "flight.osd.flyc_state")
                .filter(|v| (0.0..=255.0).contains(v))
                .map(|v| format!("{:?}", FlightMode::from(v as u8)))
                .or_else(|| col_map.get_str(&row, "elrs.flight_mode").map(str::to_string))
            {
                flight_mode = Some(mode);
            }
            if home.is_none() {
                home = fix(
                    col_map.get_f64(&row, "flight.home.lat_deg"),
                    col_map.get_f64(&row, "flight.home.lon_deg"),
                );
            }

            let Some(time) = col_map
                .get_str(&row, "timestamp")
                .and_then(|s| DateTime::parse_from_rfc3339(s).ok())
                .map(|t| t.with_timezone(&Utc))
            else {
                continue;
            };

            let Some((lat, lon)) = fix(
                col_map.get_f64(&row, "flight.osd.lat_deg"),
                col_map.get_f64(&row, "flight.osd.lon_deg"),
            )
            .or_else(|| {
                fix(
                    col_map.get_f64(&row, "elrs.gps.lat_deg"),
                    col_map.get_f64(&row, "elrs.gps.lon_deg"),
                )
            }) else {
                continue;
            };

            let start = *start_time.get_or_insert(time);
            let mut point = self.parse_row(&col_map, &row);
            point.timestamp_ms = (time - start).num_milliseconds();
            point.latitude = Some(lat);
            point.longitude = Some(lon);
            point.flight_mode = flight_mode.clone();
            points.push(point);
        }

        if points.is_empty() {
            return Err(ParserError::NoTelemetryData);
        }

        let mut stats = LogParser::new(self.db).calculate_stats(&points);
        if home.is_some() {
            stats.home_location = home.map(|(lat, lon)| [lon, lat]);
        }

        let file_name = file_path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown")
            .to_string();
        let display_name = file_path
            .file_stem()
            .and_then(|s| s.to_str())
            .filter(|s| !s.trim().is_empty())
            .unwrap_or(&file_name)
            .to_string();

        let (photo_count, video_count) = crate::models::count_media_events(&points);
        let end_time = start_time.map(|s| s + chrono::Duration::milliseconds(points.last().unwrap().timestamp_ms));

        let metadata = FlightMetadata {
            id: self.db.generate_flight_id(),
            file_name,
            display_name,
            file_hash: Some(file_hash.to_string()),
            drone_model,
            // The CSV only has product codes (e.g. ZA530 for every O4 Air Unit), no serials
            drone_serial: None,
            aircraft_name: None,
            battery_serial: None,
            cycle_count: None,
            rc_serial: None,
            battery_life: None,
            start_time,
            end_time,
            duration_secs: Some(stats.duration_secs),
            total_distance: Some(stats.total_distance_m),
            max_altitude: Some(stats.max_altitude_m),
            max_speed: Some(stats.max_speed_ms),
            home_lat: stats.home_location.map(|h| h[1]),
            home_lon: stats.home_location.map(|h| h[0]),
            point_count: points.len() as i32,
            photo_count,
            video_count,
        };

        log::info!(
            "DJI goggles parse complete in {:.1}s: duration={:.1}s, distance={:.0}m, max_alt={:.1}m, points={}",
            parse_start.elapsed().as_secs_f64(),
            stats.duration_secs,
            stats.total_distance_m,
            stats.max_altitude_m,
            points.len()
        );

        let mut tags = LogParser::generate_smart_tags(&metadata, &stats);
        tags.insert(0, "DJI Goggles".to_string());

        Ok(ParseResult { metadata, points, tags, manual_tags: Vec::new(), notes: None, color: None, messages: Vec::new() })
    }

    /// Map one row to a telemetry point. Position, time and flight mode are set by the caller.
    fn parse_row(&self, col_map: &ColumnMap, row: &[String]) -> TelemetryPoint {
        let f = |field: &str| col_map.get_f64(row, field);
        let first = |fields: &[&str]| col_map.first_f64(row, fields);

        // Height above takeoff. Betaflight 4.5 sends its altitude estimate
        // (relative to arming) in the CRSF GPS frame, so the ELRS value is
        // relative too.
        let height = first(&["flight.osd.rel_h_m", "elrs.gps.alt_m"]);

        let speed = match (f("flight.osd.vgx_mps"), f("flight.osd.vgy_mps")) {
            (Some(x), Some(y)) => Some(x.hypot(y)),
            _ => f("elrs.gps.speed_mps"),
        };

        let cell_voltages = col_map
            .get_str(row, "battery.cells.cell_mv_list")
            .map(|list| {
                list.split(|c: char| !c.is_ascii_digit() && c != '.')
                    .filter_map(|v| v.parse::<f64>().ok())
                    .filter(|&mv| mv > 0.0)
                    .map(|mv| mv / 1000.0)
                    .collect::<Vec<f64>>()
            })
            .filter(|cells| !cells.is_empty());

        TelemetryPoint {
            altitude: height,
            height,

            speed,
            velocity_x: f("flight.osd.vgx_mps"),
            velocity_y: f("flight.osd.vgy_mps"),
            velocity_z: f("flight.osd.vgz_mps"),

            pitch: first(&["flight.osd.pitch_deg", "elrs.att.pitch_deg"]),
            roll: first(&["flight.osd.roll_deg", "elrs.att.roll_deg"]),
            yaw: first(&["flight.osd.yaw_deg", "elrs.att.yaw_deg"]),

            battery_percent: first(&["battery.dynamic.soc", "elrs.battery.remaining_pct"])
                .filter(|&v| (0.0..=100.0).contains(&v))
                .map(|v| v.round() as i32),
            battery_voltage: first(&["battery.dynamic.voltage_mv", "elrs.battery.voltage_mv"])
                .filter(|&mv| mv > 0.0)
                .map(|mv| mv / 1000.0),
            battery_current: first(&["battery.dynamic.current_ma", "elrs.battery.current_ma"])
                .map(|ma| ma / 1000.0),
            battery_temp: f("battery.dynamic.temp_c"),
            battery_full_capacity: f("battery.dynamic.full_cap_mah").filter(|&v| v > 0.0),
            battery_remained_capacity: f("battery.dynamic.remain_cap_mah").filter(|&v| v > 0.0),
            cell_voltages,

            gps_signal: f("flight.osd.gps_level").map(|v| v as i32),
            satellites: first(&["flight.osd.gps_nums", "elrs.gps.sats"]).map(|v| v as i32),
            // ELRS link quality as measured at the receiver is the control
            // uplink; the goggles' own video link quality is the downlink.
            rc_uplink: f("elrs.link.rqly_pct").map(|v| v as i32),
            rc_downlink: f("link.signal_quality").map(|v| v as i32),

            // Non-zero record state is recording, as in DJI .txt logs; the record
            // clock only runs while recording
            is_video: f("camera.state.record_state")
                .map(|v| v != 0.0)
                .or_else(|| f("camera.state.video_record_time").map(|t| t > 0.0)),

            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    const HEADER: &str = "timestamp,flight.osd.lon_deg,flight.osd.lat_deg,flight.osd.rel_h_m,\
flight.osd.vgx_mps,flight.osd.vgy_mps,flight.osd.flyc_state,flight.home.lat_deg,flight.home.lon_deg,\
camera.state.record_state,camera.state.video_record_time,\
link.goggles.name,link.aircraft.name,link.signal_quality,\
elrs.gps.lat_deg,elrs.gps.lon_deg,elrs.gps.alt_m,elrs.gps.speed_mps,\
elrs.battery.voltage_mv,elrs.battery.current_ma,elrs.battery.remaining_pct,\
elrs.link.rqly_pct,elrs.flight_mode";

    fn write_csv(dir: &Path, name: &str, body: &str) -> std::path::PathBuf {
        let path = dir.join(name);
        let mut f = File::create(&path).unwrap();
        writeln!(f, "{}", HEADER).unwrap();
        f.write_all(body.as_bytes()).unwrap();
        path
    }

    #[test]
    fn detects_goggles_csv_only() {
        let dir = tempfile::tempdir().unwrap();
        let goggles = write_csv(dir.path(), "telemetry_20260417_193638_492.csv", "");
        assert!(GogglesParser::is_goggles_csv(&goggles));

        let other = dir.path().join("other.csv");
        std::fs::write(&other, "time_s,lat,lng,alt_m,height_m\n").unwrap();
        assert!(!GogglesParser::is_goggles_csv(&other));
    }

    #[test]
    fn maps_elrs_and_dji_rows() {
        let dir = tempfile::tempdir().unwrap();
        let path = write_csv(
            dir.path(),
            "telemetry_20260417_193638_492.csv",
            concat!(
                // Before the air unit links: a stale fix, dropped
                "2026-04-17T19:36:37.518+02:00,,,,,,,48.6,16.5,0,0,Goggles Integra,,0,48.6,16.5,-7,0,7300,900,62,100,OLD\n",
                // Linked, no fix yet: skipped
                "2026-04-17T19:36:38.518+02:00,,,,,,,,,0,0,Goggles Integra,O4 Air Unit,0,,,,,7300,900,62,100,\n",
                // ELRS-only quad
                "2026-04-17T19:36:38.719+02:00,,,,,,,,,0,0,Goggles Integra,O4 Air Unit,90,48.5,16.46,12,3.5,7200,1800,61,99,\n",
                // DJI flight controller wins over ELRS; recording with the clock still at 0
                "2026-04-17T19:36:39.719+02:00,16.47,48.51,30,3,4,6,48.49,16.45,2,0,Goggles Integra,O4 Air Unit,70,48.5,16.46,12,3.5,7100,1800,60,98,ACRO\n",
                // Blank mode: the last one carries forward
                "2026-04-17T19:36:40.719+02:00,,,,,,,,,0,0,Goggles Integra,O4 Air Unit,90,48.5,16.46,12,3.5,7200,1800,61,99,\n",
            ),
        );
        let db = Database::new(dir.path().to_path_buf(), "default").unwrap();
        let res = GogglesParser::new(&db).parse(&path, "hash").unwrap();

        assert_eq!(res.points.len(), 3, "pre-link and no-fix rows are dropped");
        let (a, b, c) = (&res.points[0], &res.points[1], &res.points[2]);
        assert_eq!(a.timestamp_ms, 0);
        assert_eq!(b.timestamp_ms, 1000);
        assert_eq!((a.latitude, a.longitude, a.height), (Some(48.5), Some(16.46), Some(12.0)));
        assert_eq!(a.speed, Some(3.5));
        assert_eq!(a.battery_voltage, Some(7.2));
        assert_eq!(a.battery_current, Some(1.8));
        assert_eq!(a.battery_percent, Some(61));
        assert_eq!((a.rc_uplink, a.rc_downlink), (Some(99), Some(90)));
        assert_eq!(a.flight_mode, None, "pre-link mode is discarded");
        assert_eq!(a.is_video, Some(false));

        assert_eq!((b.latitude, b.longitude, b.height), (Some(48.51), Some(16.47), Some(30.0)));
        assert_eq!(b.speed, Some(5.0));
        assert_eq!(b.flight_mode.as_deref(), Some("GPSAtti"), "DJI flyc_state wins over ELRS");
        assert_eq!(b.is_video, Some(true));
        assert_eq!(c.flight_mode.as_deref(), Some("GPSAtti"), "mode carries forward");

        let meta = &res.metadata;
        assert_eq!((meta.home_lat, meta.home_lon), (Some(48.49), Some(16.45)), "pre-link home is discarded");
        assert_eq!(meta.drone_model.as_deref(), Some("DJI O4 Air Unit"));
        assert_eq!(meta.start_time.unwrap().to_rfc3339(), "2026-04-17T17:36:38.719+00:00");
        assert_eq!(meta.duration_secs, Some(2.0));
        assert_eq!(meta.video_count, 1);
        assert_eq!(res.tags.first().map(String::as_str), Some("DJI Goggles"));
    }
}
