//! Pulls raw frames from `mediad`'s `media.frame` endpoint into the dataflow.
//!
//! `mediad` owns the camera and serves a local, on-demand raw frame socket:
//! a JSON-RPC request is answered by a `MediaFrameHeader` line followed by
//! exactly `bytes` raw UYVY bytes. This node asks for one frame per 33 ms tick
//! and republishes it as an Arrow `UInt8` array.
//!
//! The payload is written **directly into the dora sample**: the Arrow IPC
//! header is encoded into the sample first, then the socket's binary tail is
//! read straight into its data region. There is no intermediate `Vec`, and for
//! frames above dora's zero-copy threshold the sample lives in shared memory,
//! so downstream nodes decode it without another copy.
//!
//! `MEDIA_FRAME_SOCKET` names the endpoint: the simulated duck sets it to
//! `<state>/duck-a-frame.sock`; a real robot uses `/run/mediad/media.sock`.
//! Without it the node logs and publishes nothing, like the state node.

use dora_node_api::{
    DoraNode, Event, MetadataParameters, Parameter,
    arrow_utils::ipc_encode,
    dora_core::config::DataId,
    metadata::{FRAMING, FRAMING_ARROW_IPC},
};
use duck_ipc_proto::{MediaFrameHeader, Response};
use eyre::{Result, WrapErr, eyre};
use std::io::{self, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;

const TICK: &str = "tick";
const IMAGE: &str = "image";
/// `media.frame` answers with one JSON line, then raw bytes.
const MAX_RESPONSE_LINE: usize = 4096;
/// The endpoint already refuses anything above 16 MiB; keep the same bound here.
const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

struct FrameSource {
    path: Option<PathBuf>,
    stream: Option<UnixStream>,
    next_id: u64,
    failures: u32,
}

impl FrameSource {
    fn from_env() -> Self {
        let path = std::env::var_os("MEDIA_FRAME_SOCKET").map(PathBuf::from);
        if path.is_none() {
            eprintln!("dora-camera: MEDIA_FRAME_SOCKET is not set; no frames will be published");
        }
        Self {
            path,
            stream: None,
            next_id: 0,
            failures: 0,
        }
    }

    fn connect(&mut self) -> Result<()> {
        let path = self.path.clone().expect("next_frame checks the path");
        let stream = UnixStream::connect(&path)
            .wrap_err_with(|| format!("connect to mediad at {}", path.display()))?;
        self.stream = Some(stream);
        self.failures = 0;
        eprintln!("dora-camera: connected to {}", path.display());
        Ok(())
    }

    fn disconnect(&mut self) {
        self.stream = None;
    }

    /// Ask for the next captured frame. On success the connection is
    /// positioned at the raw payload, which the caller must read before the
    /// next request.
    fn next_frame(&mut self) -> Option<MediaFrameHeader> {
        self.path.as_ref()?;
        if self.stream.is_none()
            && let Err(err) = self.connect()
        {
            self.note_failure(err);
            return None;
        }

        match self.request() {
            Ok(header) => header,
            Err(err) => {
                // A failed request may leave an unread payload on the stream.
                // Reconnecting is cheaper than trying to resynchronise it.
                self.stream = None;
                self.note_failure(err);
                None
            }
        }
    }

    fn request(&mut self) -> Result<Option<MediaFrameHeader>> {
        let stream = self.stream.as_mut().expect("connected above");
        self.next_id += 1;
        let request = serde_json::json!({
            "jsonrpc": "2.0",
            "id": self.next_id,
            "method": duck_ipc_proto::method::MEDIA_FRAME,
        });
        let line = request.to_string() + "\n";
        stream
            .write_all(line.as_bytes())
            .wrap_err("send media.frame request")?;

        let line = read_line(stream, MAX_RESPONSE_LINE).wrap_err("read media.frame response")?;
        let response: Response =
            serde_json::from_str(&line).wrap_err("malformed media.frame response")?;
        if let Some(error) = response.error {
            eprintln!("dora-camera: mediad refused the frame: {error:?}");
            return Ok(None);
        }

        let result = response
            .result
            .ok_or_else(|| eyre!("media.frame response without a result"))?;
        let header: MediaFrameHeader =
            serde_json::from_value(result).wrap_err("failed to parse the frame header")?;
        if !header.valid_uyvy() || header.bytes > MAX_FRAME_BYTES {
            // The header is untrustworthy, so the payload length is too. Drop
            // the connection rather than try to discard a guessed byte count.
            self.stream = None;
            return Err(eyre!("invalid frame header: {header:?}"));
        }

        Ok(Some(header))
    }

    fn read_payload(&mut self, out: &mut [u8]) -> Result<()> {
        let stream = self.stream.as_mut().expect("request returned a frame");
        stream
            .read_exact(out)
            .wrap_err("read the frame payload from mediad")
    }

    fn note_failure(&mut self, err: eyre::Report) {
        self.failures += 1;
        // A camera that is down must not fill the log at 30 lines a second.
        if self.failures == 1 || self.failures.is_multiple_of(100) {
            eprintln!("dora-camera: {err} (attempt {})", self.failures);
        }
    }
}

/// Read one newline-terminated line without reading into the payload.
///
/// A `BufReader` here would be a bug: the response is a line followed
/// immediately by raw bytes, and readahead would swallow the start of the
/// frame. One byte at a time is fine for a header of a few hundred bytes.
fn read_line(stream: &mut UnixStream, max: usize) -> io::Result<String> {
    let mut line = Vec::with_capacity(256);
    let mut byte = [0u8; 1];
    while line.len() < max {
        stream.read_exact(&mut byte)?;
        if byte[0] == b'\n' {
            return String::from_utf8(line)
                .map_err(|err| io::Error::new(io::ErrorKind::InvalidData, err));
        }
        line.push(byte[0]);
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        "media.frame response line too long",
    ))
}

fn frame_parameters(header: &MediaFrameHeader) -> MetadataParameters {
    let mut parameters = MetadataParameters::new();
    parameters.insert(
        "width".to_owned(),
        Parameter::Integer(i64::from(header.width)),
    );
    parameters.insert(
        "height".to_owned(),
        Parameter::Integer(i64::from(header.height)),
    );
    parameters.insert(
        "format".to_owned(),
        Parameter::String(header.format.clone()),
    );
    parameters.insert("bytes".to_owned(), Parameter::Integer(header.bytes as i64));
    parameters.insert(
        "rotate".to_owned(),
        Parameter::Integer(i64::from(header.rotate)),
    );
    parameters.insert(
        "captured_at_unix_us".to_owned(),
        Parameter::String(header.captured_at_unix_us.to_string()),
    );
    parameters
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    // Without a subscriber, dora's own large-send diagnostics (the
    // `zenoh direct path` / `daemon path` lines) are dropped. They are the
    // only way to tell from outside whether the frame took the zero-copy path.
    // `dora_node_api::init_tracing` needs a tokio runtime; this node is a plain
    // synchronous loop, so install the warn-level subscriber directly.
    let _ = tracing_subscriber::fmt()
        .with_max_level(tracing_subscriber::filter::LevelFilter::WARN)
        .with_ansi(false)
        .with_writer(std::io::stderr)
        .try_init();
    let mut source = FrameSource::from_env();
    let mut published = 0_u64;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } if id.as_str() == TICK => {
                let Some(header) = source.next_frame() else {
                    continue;
                };

                let total = ipc_encode::uint8_ipc_len(header.bytes)?;
                let mut sample = node.allocate_data_sample(total)?;
                let buffer: &mut [u8] = &mut sample;
                let offset = ipc_encode::encode_uint8_ipc_header(buffer, header.bytes)?;

                if let Err(err) = source.read_payload(&mut buffer[offset..offset + header.bytes]) {
                    // The sample is not sent when the read fails, so a short
                    // frame never reaches a consumer.
                    source.disconnect();
                    eprintln!("dora-camera: {err}; reconnecting");
                    continue;
                }

                let mut parameters = frame_parameters(&header);
                parameters.insert(
                    FRAMING.to_owned(),
                    Parameter::String(FRAMING_ARROW_IPC.to_owned()),
                );
                node.send_output_sample(DataId::from(IMAGE.to_owned()), parameters, Some(sample))?;

                published += 1;
                if published == 1 || published.is_multiple_of(30) {
                    eprintln!(
                        "dora-camera: frame {published}: {}x{} {} rotate={} bytes={}",
                        header.width, header.height, header.format, header.rotate, header.bytes
                    );
                }
            }
            Event::Input { id, .. } => eprintln!("dora-camera: ignoring input `{id}`"),
            _ => {}
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::{get_integer_param, get_string_param};

    fn header() -> MediaFrameHeader {
        MediaFrameHeader {
            width: 2,
            height: 1,
            format: "UYVY".to_owned(),
            bytes: 4,
            captured_at_unix_us: 42,
            rotate: 90,
        }
    }

    #[test]
    fn reads_a_line_without_swallowing_the_payload() {
        let (mut writer, mut reader) = UnixStream::pair().unwrap();
        writer.write_all(b"{\"a\":1}\nPAYLOAD").unwrap();

        let line = read_line(&mut reader, MAX_RESPONSE_LINE).unwrap();
        assert_eq!(line, "{\"a\":1}");

        let mut rest = [0u8; 7];
        reader.read_exact(&mut rest).unwrap();
        assert_eq!(&rest, b"PAYLOAD");
    }

    #[test]
    fn rejects_a_line_that_never_ends() {
        let (mut writer, mut reader) = UnixStream::pair().unwrap();
        writer.write_all(b"1234567890").unwrap();

        let err = read_line(&mut reader, 4).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }

    #[test]
    fn carries_frame_metadata() {
        let parameters = frame_parameters(&header());
        assert_eq!(get_integer_param(&parameters, "width"), Some(2));
        assert_eq!(get_integer_param(&parameters, "height"), Some(1));
        assert_eq!(get_string_param(&parameters, "format"), Some("UYVY"));
        assert_eq!(get_integer_param(&parameters, "rotate"), Some(90));
        assert_eq!(
            get_string_param(&parameters, "captured_at_unix_us"),
            Some("42")
        );
    }

    #[test]
    fn rejects_a_frame_whose_bytes_do_not_match_its_geometry() {
        let mut header = header();
        header.bytes = 3;
        assert!(!header.valid_uyvy());
    }
}
