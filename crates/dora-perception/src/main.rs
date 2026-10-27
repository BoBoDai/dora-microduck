//! Finds a coloured target in `camera`'s UYVY frames.
//!
//! This is the first perception slice, deliberately not a model: it thresholds
//! the chroma plane for one saturated orange/red object, takes the centroid of
//! the matching pixels, and publishes the target's bearing and size. That is
//! enough for the reactive brain to turn towards it, walk, and stop when it
//! gets close. A detector (NPU or otherwise) can replace `detect` without
//! changing the message contract.
//!
//! `target` is an Arrow `f32[3]`:
//!
//! ```text
//! [visible (0 or 1), bearing (-1 = left, +1 = right), area (fraction of pixels)]
//! ```
//!
//! The frame is **not rotated**. `mediad` reports the mount angle in `rotate`,
//! and the raw sensor on the duck is mounted 90° clockwise from upright. The
//! node maps each pixel's raw coordinates into the upright image before taking
//! the centroid, so steering is correct without rotating 460 KB per frame:
//!
//! ```text
//! rotate 90:  x_upright = height - 1 - y_raw
//! rotate 270: x_upright = y_raw
//! rotate 180: x_upright = width - 1 - x_raw
//! ```
//!
//! Bearing is positive to the **right** of the duck. The brain maps that to
//! `vyaw` (positive is left, per `duck_ipc_proto::MoveParams`).
//!
//! Thresholds and the minimum blob size are tunable with
//! `PERCEPTION_V_MIN`, `PERCEPTION_U_MAX`, `PERCEPTION_Y_MIN` and
//! `PERCEPTION_MIN_AREA` (a fraction, default 0.001).

use dora_node_api::{
    DoraNode, Event,
    arrow_v59::array::{Array as _, UInt8Array},
    dora_core::config::DataId,
    get_integer_param, get_string_param,
};
use eyre::Result;

const IMAGE: &str = "image";
const TARGET: &str = "target";

#[derive(Clone, Copy, Debug, PartialEq)]
struct Thresholds {
    v_min: i16,
    u_max: i16,
    y_min: i16,
    min_area: f32,
}

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            // Orange (RGB 255, 140, 0) is about U=40, V=197, Y=158. The floor,
            // sky and white markings are close to U=V=128, so the chroma
            // difference is the signal; the luma floor rejects dark shadows.
            v_min: 150,
            u_max: 120,
            y_min: 50,
            min_area: 0.001,
        }
    }
}

impl Thresholds {
    fn from_env() -> Self {
        let defaults = Self::default();
        Self {
            v_min: env_byte("PERCEPTION_V_MIN", defaults.v_min),
            u_max: env_byte("PERCEPTION_U_MAX", defaults.u_max),
            y_min: env_byte("PERCEPTION_Y_MIN", defaults.y_min),
            min_area: env_fraction("PERCEPTION_MIN_AREA", defaults.min_area),
        }
    }
}

fn env_byte(name: &str, default: i16) -> i16 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<i16>().ok())
        .unwrap_or(default)
        .clamp(0, 255)
}

fn env_fraction(name: &str, default: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(default)
        .clamp(0.0, 1.0)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Detection {
    visible: bool,
    bearing: f32,
    area: f32,
}

impl Detection {
    const NONE: Self = Self {
        visible: false,
        bearing: 0.0,
        area: 0.0,
    };

    fn values(self) -> Vec<f32> {
        vec![
            if self.visible { 1.0 } else { 0.0 },
            self.bearing,
            self.area,
        ]
    }
}

/// Horizontal coordinate in the upright image for a raw pixel.
fn upright_x(x: usize, y: usize, width: usize, height: usize, rotate: u32) -> usize {
    match rotate % 360 {
        90 => height - 1 - y,
        180 => width - 1 - x,
        270 => y,
        _ => x,
    }
}

/// Width of the upright image. A quarter turn swaps the axes; an angle the
/// node does not understand is treated as no rotation, matching `upright_x`.
fn upright_width(width: usize, height: usize, rotate: u32) -> usize {
    match rotate % 360 {
        90 | 270 => height,
        _ => width,
    }
}

/// Centroid and area of the pixels that pass the orange threshold.
fn detect(
    bytes: &[u8],
    width: usize,
    height: usize,
    rotate: u32,
    thresholds: Thresholds,
) -> Detection {
    if width < 2 || height == 0 {
        return Detection::NONE;
    }

    let stride = width
        .checked_mul(2)
        .expect("a frame width cannot overflow the byte count");
    let expected = stride
        .checked_mul(height)
        .expect("a frame height cannot overflow the byte count");
    if bytes.len() < expected {
        return Detection::NONE;
    }

    let mut count = 0_u64;
    let mut sum_x = 0_f64;

    // UYVY packs two pixels into four bytes: U, Y0, V, Y1. Both pixels share
    // the chroma, so the colour test is done once per pair.
    let pairs = width / 2;
    for y in 0..height {
        let row = y * stride;
        for pair in 0..pairs {
            let i = row + pair * 4;
            let u = i16::from(bytes[i]);
            let luma0 = i16::from(bytes[i + 1]);
            let v = i16::from(bytes[i + 2]);
            let luma1 = i16::from(bytes[i + 3]);

            if v < thresholds.v_min || u > thresholds.u_max {
                continue;
            }

            let x0 = pair * 2;
            let x1 = x0 + 1;
            if luma0 >= thresholds.y_min {
                count += 1;
                sum_x += upright_x(x0, y, width, height, rotate) as f64;
            }
            if luma1 >= thresholds.y_min {
                count += 1;
                sum_x += upright_x(x1, y, width, height, rotate) as f64;
            }
        }
    }

    let area = count as f32 / (width * height) as f32;
    if count == 0 || area < thresholds.min_area {
        return Detection::NONE;
    }

    let upright = upright_width(width, height, rotate) as f64;
    // Pixel centers are at x + 0.5; the image center is at upright / 2.
    let center = (sum_x + count as f64 * 0.5) / count as f64;
    let bearing = (2.0 * center / upright - 1.0).clamp(-1.0, 1.0);

    Detection {
        visible: true,
        bearing: bearing as f32,
        area,
    }
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let thresholds = Thresholds::from_env();
    eprintln!(
        "dora-perception: orange threshold V>={}, U<={}, Y>={}, min_area={:.4}",
        thresholds.v_min, thresholds.u_max, thresholds.y_min, thresholds.min_area
    );

    let mut frames = 0_u64;
    let mut invalid = 0_u64;
    let mut last_visible = None;

    while let Some(event) = events.recv() {
        let Event::Input { id, metadata, data } = event else {
            continue;
        };
        if id.as_str() != IMAGE {
            eprintln!("dora-perception: ignoring input `{id}`");
            continue;
        }

        let array = data.as_array();
        let Some(bytes) = array.as_any().downcast_ref::<UInt8Array>() else {
            eprintln!("dora-perception: image is not a Uint8 array");
            continue;
        };

        let width = get_integer_param(&metadata.parameters, "width").unwrap_or(0);
        let height = get_integer_param(&metadata.parameters, "height").unwrap_or(0);
        let rotate = get_integer_param(&metadata.parameters, "rotate").unwrap_or(0);
        let format = get_string_param(&metadata.parameters, "format").unwrap_or("?");

        if format != "UYVY" || width <= 0 || height <= 0 {
            invalid += 1;
            if invalid == 1 || invalid.is_multiple_of(100) {
                eprintln!(
                    "dora-perception: cannot read {format} {width}x{height} frame (attempt {invalid})"
                );
            }
            continue;
        }

        let detection = detect(
            bytes.values(),
            width as usize,
            height as usize,
            rotate.rem_euclid(360) as u32,
            thresholds,
        );

        if last_visible != Some(detection.visible) || frames.is_multiple_of(30) {
            eprintln!(
                "dora-perception: frame {}: visible={} bearing={:+.3} area={:.4}",
                frames + 1,
                detection.visible,
                detection.bearing,
                detection.area
            );
            last_visible = Some(detection.visible);
        }

        node.send_output(
            DataId::from(TARGET.to_owned()),
            metadata.parameters,
            detection.values(),
        )?;
        frames += 1;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const BACKGROUND: [u8; 4] = [128, 80, 128, 80]; // U, Y0, V, Y1: neutral blue-grey
    const ORANGE: [u8; 4] = [40, 160, 200, 160];

    fn blank(width: usize, height: usize) -> Vec<u8> {
        let mut frame = Vec::with_capacity(width * height * 2);
        for _ in 0..width * height / 2 {
            frame.extend_from_slice(&BACKGROUND);
        }
        frame
    }

    /// Paint whole UYVY pairs, so `pair_start..pair_end` covers both pixels.
    fn paint_pairs(
        frame: &mut [u8],
        width: usize,
        pair_start: usize,
        pair_end: usize,
        y_start: usize,
        y_end: usize,
        value: [u8; 4],
    ) {
        for y in y_start..y_end {
            for pair in pair_start..pair_end {
                let i = (y * width + pair * 2) * 2;
                frame[i..i + 4].copy_from_slice(&value);
            }
        }
    }

    fn thresholds() -> Thresholds {
        Thresholds {
            min_area: 0.0,
            ..Thresholds::default()
        }
    }

    #[test]
    fn finds_a_blob_and_reports_its_area() {
        let (width, height) = (8, 4);
        let mut frame = blank(width, height);
        paint_pairs(&mut frame, width, 0, 2, 1, 3, ORANGE);

        let detection = detect(&frame, width, height, 0, thresholds());
        assert!(detection.visible);
        assert_eq!(detection.area, 8.0 / 32.0);
        // x pixels 0..4; centers 0.5..3.5 average 2.0; bearing 2*2/8-1 = -0.5.
        assert!(detection.bearing < -0.49 && detection.bearing > -0.51);
    }

    #[test]
    fn a_quarter_turn_maps_raw_top_to_upright_right() {
        let (width, height) = (8, 4);
        let mut frame = blank(width, height);
        // Raw row 0 becomes upright x = height - 1 = 3, the right edge.
        paint_pairs(&mut frame, width, 0, 2, 0, 1, ORANGE);
        let detection = detect(&frame, width, height, 90, thresholds());
        assert!(detection.visible);
        assert!(detection.bearing > 0.7, "{detection:?}");

        let mut bottom = blank(width, height);
        paint_pairs(&mut bottom, width, 0, 2, 3, 4, ORANGE);
        let detection = detect(&bottom, width, height, 90, thresholds());
        assert!(detection.bearing < -0.7, "{detection:?}");
    }

    #[test]
    fn other_rotations_map_the_expected_edge() {
        let (width, height) = (8, 4);

        let mut left = blank(width, height);
        paint_pairs(&mut left, width, 0, 1, 1, 3, ORANGE);
        // x=0,1 -> upright 7,6; center 7.0; bearing 2*7/8-1 = 0.75.
        assert!(detect(&left, width, height, 180, thresholds()).bearing > 0.7);

        let mut top = blank(width, height);
        paint_pairs(&mut top, width, 0, 2, 0, 1, ORANGE);
        // y=0 -> upright x=0; center 0.5; bearing 2*0.5/4-1 = -0.75.
        assert!(detect(&top, width, height, 270, thresholds()).bearing < -0.7);

        // An unsupported angle falls back to no rotation rather than swapping
        // the axes without rotating the coordinates.
        assert_eq!(upright_width(width, height, 45), width);
    }

    #[test]
    fn ignores_a_neutral_frame() {
        let detection = detect(&blank(8, 4), 8, 4, 90, thresholds());
        assert_eq!(detection, Detection::NONE);
    }

    #[test]
    fn ignores_a_blob_below_the_minimum_area() {
        let (width, height) = (8, 4);
        let mut frame = blank(width, height);
        paint_pairs(&mut frame, width, 0, 2, 0, 1, ORANGE);
        let thresholds = Thresholds {
            min_area: 0.5,
            ..Thresholds::default()
        };
        assert_eq!(
            detect(&frame, width, height, 0, thresholds),
            Detection::NONE
        );
    }

    #[test]
    fn rejects_chroma_outside_the_target_range() {
        let (width, height) = (8, 4);
        let mut frame = blank(width, height);
        // High luma, but U is too high and V too low to be orange.
        paint_pairs(&mut frame, width, 0, 2, 1, 3, [130, 200, 140, 200]);
        assert_eq!(
            detect(&frame, width, height, 0, thresholds()),
            Detection::NONE
        );
    }

    #[test]
    fn output_is_three_floats() {
        assert_eq!(Detection::NONE.values(), vec![0.0, 0.0, 0.0]);
        let values = Detection {
            visible: true,
            bearing: 0.25,
            area: 0.5,
        }
        .values();
        assert_eq!(values, vec![1.0, 0.25, 0.5]);
    }
}
