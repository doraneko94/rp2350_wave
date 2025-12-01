#[derive(Clone, Copy)]
pub struct Delta {
    pub sin: f64,
    pub cos: f64,
}

// C4 261.626 Hz
pub const C4: Delta = Delta { sin: 0.0016438438988365322, cos: 0.9999986488877054 };
// D4 293.655 Hz
pub const D4: Delta = Delta { sin: 0.0018450877344911977, cos: 0.9999982978241773 };
// E4 311.127 Hz
pub const E4: Delta = Delta { sin: 0.001954867349975055, cos: 0.9999980892449966 };
// F4 349.228 Hz
pub const F4: Delta = Delta { sin: 0.0021942624776338554, cos: 0.9999975926031919 };
// G4 391.995 Hz
pub const G4: Delta = Delta { sin: 0.002462974734313224, cos: 0.9999969668731291 };
// A4 440.000 Hz
pub const A4: Delta = Delta { sin: 0.0027645980135088, cos: 0.9999961784916099 };
// B4 493.883 Hz
pub const B4: Delta = Delta { sin: 0.003103153428709884, cos: 0.9999951852078078 };
// C5 523.251 Hz
pub const C5: Delta = Delta { sin: 0.00328767707248631, cos: 0.9999945955751296 };
// D5 587.330 Hz
pub const D5: Delta = Delta { sin: 0.0036902948505054397, cos: 0.9999931908387758 };

pub fn decode_delta_duration(note: (Delta, u8), output_hz: u32, measure_hz: u32) -> (f64, f64, u32) {
    let (tone, length) = note;
    let duration = output_hz as f64 / measure_hz as f64 / length as f64;

    (tone.sin, tone.cos, duration as u32)
}