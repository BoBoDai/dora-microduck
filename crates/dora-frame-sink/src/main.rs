//! Debug consumer for `dora-camera`.
//!
//! It reads the `image` array in place — `as_array()` and `values()` alias the
//! shared-memory buffer — and logs what arrived, so the camera path can be
//! verified before `perception` exists. It is deliberately a separate node:
//! the next slice replaces it with the real perception node, and until then it
//! is also what gives the camera output a subscriber.
//!
//! Logging every frame would be 30 lines a second; one in thirty is enough to
//! see the frame size and metadata are stable. The checksum samples every
//! 4096th byte so the sink itself does not become the CPU cost being measured.
//!
//! Set `FRAME_DUMP_DIR` to write the frames this node actually receives. Each
//! dump is a raw `.uyvy` file plus a `.json` sidecar carrying width, height,
//! format, mount rotation and capture time. `FRAME_DUMP_EVERY` (default 30)
//! and `FRAME_DUMP_LIMIT` (default 10; `0` means unlimited) bound it, because
//! 30 frames a second at 460 KB is 14 MB/s.
//!
//! The raw frame is unrotated: `rotate` in the sidecar says how far the camera
//! is mounted from upright. For the simulated duck that is 90°, so:
//!
//! ```text
//! ffmpeg -f rawvideo -pix_fmt uyvy422 -s 640x360 \
//!        -i frame-000001.uyvy -vf transpose=1 frame.png
//! ```

use dora_node_api::{
    DoraNode, Event, MetadataParameters,
    arrow_v59::array::{Array as _, UInt8Array},
    get_integer_param, get_string_param,
};
use eyre::{Result, WrapErr};
use std::path::PathBuf;

const IMAGE: &str = "image";

/// Bounded raw-frame dumping, enabled only by `FRAME_DUMP_DIR`.
struct Dump {
    dir: PathBuf,
    every: u64,
    limit: u64,
    written: u64,
}

impl Dump {
    fn from_env() -> Option<Self> {
        let dir = std::env::var_os("FRAME_DUMP_DIR").map(PathBuf::from)?;
        let every = std::env::var("FRAME_DUMP_EVERY")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .filter(|every| *every > 0)
            .unwrap_or(30);
        let limit = std::env::var("FRAME_DUMP_LIMIT")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .unwrap_or(10);

        if let Err(err) = std::fs::create_dir_all(&dir) {
            eprintln!(
                "dora-frame-sink: cannot create {}: {err}; frame dumping disabled",
                dir.display()
            );
            return None;
        }
        eprintln!(
            "dora-frame-sink: dumping frames to {} (every {every}, limit {})",
            dir.display(),
            if limit == 0 {
                "unlimited".to_owned()
            } else {
                limit.to_string()
            }
        );

        Some(Self {
            dir,
            every,
            limit,
            written: 0,
        })
    }

    /// The first frame is always interesting; after it, every `every`th frame,
    /// up to `limit` dumps (0 = unlimited).
    fn should_dump(&self, frame: u64) -> bool {
        (self.limit == 0 || self.written < self.limit)
            && (frame == 1 || frame.is_multiple_of(self.every))
    }

    fn write(&mut self, frame: u64, values: &[u8], metadata: &MetadataParameters) -> Result<()> {
        let path = self.dir.join(format!("frame-{frame:06}.uyvy"));
        std::fs::write(&path, values).wrap_err_with(|| format!("write {}", path.display()))?;

        let sidecar = serde_json::json!({
            "frame": frame,
            "width": get_integer_param(metadata, "width"),
            "height": get_integer_param(metadata, "height"),
            "format": get_string_param(metadata, "format"),
            "rotate": get_integer_param(metadata, "rotate"),
            "captured_at_unix_us": get_string_param(metadata, "captured_at_unix_us"),
            "bytes": values.len(),
        });
        let sidecar_path = path.with_extension("json");
        std::fs::write(&sidecar_path, serde_json::to_vec_pretty(&sidecar)?)
            .wrap_err_with(|| format!("write {}", sidecar_path.display()))?;

        self.written += 1;
        eprintln!(
            "dora-frame-sink: dumped frame {frame} to {} ({} bytes)",
            path.display(),
            values.len()
        );
        Ok(())
    }
}

fn main() -> Result<()> {
    // The node has no outputs; it stays alive for the life of the dataflow.
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut dump = Dump::from_env();
    let mut frames = 0_u64;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } if id.as_str() == IMAGE => {
                let array = data.as_array();
                let Some(bytes) = array.as_any().downcast_ref::<UInt8Array>() else {
                    eprintln!("dora-frame-sink: image is not a Uint8 array");
                    continue;
                };
                let values = bytes.values();

                frames += 1;
                if let Some(dump) = dump.as_mut()
                    && dump.should_dump(frames)
                    && let Err(err) = dump.write(frames, &values[..], &metadata.parameters)
                {
                    eprintln!("dora-frame-sink: frame {frames} dump failed: {err}");
                }

                if frames == 1 || frames.is_multiple_of(30) {
                    let checksum = values
                        .iter()
                        .step_by(4096)
                        .fold(0_u64, |acc, value| acc.wrapping_add(u64::from(*value)));
                    eprintln!(
                        "dora-frame-sink: frame {frames}: {} bytes, {}x{} {}, rotate={:?}, checksum={checksum}",
                        values.len(),
                        get_integer_param(&metadata.parameters, "width").unwrap_or(0),
                        get_integer_param(&metadata.parameters, "height").unwrap_or(0),
                        get_string_param(&metadata.parameters, "format").unwrap_or("?"),
                        get_integer_param(&metadata.parameters, "rotate"),
                    );
                }
            }
            Event::Input { id, .. } => eprintln!("dora-frame-sink: ignoring input `{id}`"),
            _ => {}
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::Parameter;

    fn metadata() -> MetadataParameters {
        let mut parameters = MetadataParameters::new();
        parameters.insert("width".to_owned(), Parameter::Integer(2));
        parameters.insert("height".to_owned(), Parameter::Integer(1));
        parameters.insert("format".to_owned(), Parameter::String("UYVY".to_owned()));
        parameters.insert("rotate".to_owned(), Parameter::Integer(90));
        parameters.insert(
            "captured_at_unix_us".to_owned(),
            Parameter::String("42".to_owned()),
        );
        parameters
    }

    #[test]
    fn dumps_the_first_frame_then_every_n_within_the_limit() {
        let dump = Dump {
            dir: PathBuf::from("/tmp"),
            every: 30,
            limit: 2,
            written: 0,
        };
        assert!(dump.should_dump(1));
        assert!(!dump.should_dump(2));
        assert!(dump.should_dump(30));

        let limited = Dump {
            dir: PathBuf::from("/tmp"),
            every: 1,
            limit: 2,
            written: 2,
        };
        assert!(!limited.should_dump(3));
    }

    #[test]
    fn writes_a_frame_and_a_sidecar() {
        let dir = std::env::temp_dir().join(format!("dora-frame-sink-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create test directory");

        let mut dump = Dump {
            dir: dir.clone(),
            every: 1,
            limit: 1,
            written: 0,
        };
        let values = [1_u8, 2, 3, 4];
        dump.write(1, &values, &metadata()).expect("write frame");

        assert_eq!(
            std::fs::read(dir.join("frame-000001.uyvy")).expect("read payload"),
            values
        );
        let sidecar = std::fs::read_to_string(dir.join("frame-000001.json")).expect("read sidecar");
        assert!(sidecar.contains("\"rotate\": 90"), "{sidecar}");
        assert!(sidecar.contains("\"format\": \"UYVY\""), "{sidecar}");

        let _ = std::fs::remove_dir_all(&dir);
    }
}
