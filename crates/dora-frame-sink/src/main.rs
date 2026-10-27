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

use dora_node_api::{
    DoraNode, Event,
    arrow_v59::array::{Array as _, UInt8Array},
    get_integer_param, get_string_param,
};
use eyre::Result;

const IMAGE: &str = "image";

fn main() -> Result<()> {
    // The node has no outputs; it stays alive for the life of the dataflow.
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut frames = 0_u64;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } if id.as_str() == IMAGE => {
                let array = data.as_array();
                let Some(bytes) = array.as_any().downcast_ref::<UInt8Array>() else {
                    eprintln!("dora-frame-sink: image is not a Uint8 array");
                    continue;
                };

                frames += 1;
                if frames == 1 || frames.is_multiple_of(30) {
                    let values = bytes.values();
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
