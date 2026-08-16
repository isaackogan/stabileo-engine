//! ECMAScript number formatting, because a diagnostic sentence is contract.
//!
//! `normalize.ts` builds the equilibrium-refusal message out of
//! `Number.prototype.toPrecision(4)` and `(6)`. Consumers read those sentences
//! and the golden gate compares them character for character, so the port
//! cannot approximate the formatting — it has to reproduce the algorithm in
//! ECMA-262 §21.1.3.5 exactly, including the `e+4` / `e-7` exponent spelling
//! and the tie rule (a halfway significand rounds AWAY from zero, "the larger
//! m", not to even the way Rust's own float formatting would).

/// TS `Number.prototype.toPrecision(precision)`.
pub fn to_precision(value: f64, precision: usize) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity".to_string() } else { "-Infinity".to_string() };
    }
    let sign = if value < 0.0 { "-" } else { "" };
    let magnitude = value.abs();

    // Step 3: zero is p zero digits at exponent 0.
    let (digits, exponent) = if magnitude == 0.0 {
        ("0".repeat(precision), 0_i32)
    } else {
        exact_digits(magnitude, precision)
    };

    // Steps 5-8: exponential outside [-6, precision), fixed point inside.
    if exponent < -6 || exponent >= precision as i32 {
        let mantissa = if precision == 1 {
            digits.clone()
        } else {
            format!("{}.{}", &digits[0..1], &digits[1..])
        };
        let exponent_sign = if exponent >= 0 { "+" } else { "-" };
        return format!("{sign}{mantissa}e{exponent_sign}{}", exponent.abs());
    }
    if exponent == precision as i32 - 1 {
        return format!("{sign}{digits}");
    }
    if exponent >= 0 {
        let split = exponent as usize + 1;
        return format!("{sign}{}.{}", &digits[0..split], &digits[split..]);
    }
    format!("{sign}0.{}{digits}", "0".repeat((-(exponent + 1)) as usize))
}

/// The `precision` most significant decimal digits of a positive finite f64,
/// with the exponent of the leading one, rounded half-away-from-zero.
///
/// Rust's `{:.*e}` is exact to the requested digit count (it falls back to a
/// bignum path rather than the shortest-repr one), so asking for far more
/// digits than a double can distinguish and rounding the digit STRING gives
/// the spec's "m x 10^(e-p+1) closest to x, ties to the larger m" without a
/// double-rounding hazard: the guard digits below only ever decide the tie.
fn exact_digits(magnitude: f64, precision: usize) -> (String, i32) {
    const GUARD_DIGITS: usize = 40;
    let scientific = format!("{:.*e}", GUARD_DIGITS, magnitude);
    let (mantissa, exponent) = scientific.split_once('e').expect("Rust writes an exponent");
    let mut exponent: i32 = exponent.parse().expect("Rust writes a decimal exponent");
    let mut digits: Vec<u8> =
        mantissa.bytes().filter(|byte| byte.is_ascii_digit()).map(|byte| byte - b'0').collect();

    // Round at `precision`. Ties round up, which is the spec's larger m, and
    // is also why a plain `>= 5` test is the whole rule.
    if digits.len() > precision {
        let round_up = digits[precision] >= 5;
        digits.truncate(precision);
        if round_up {
            let mut index = precision;
            loop {
                if index == 0 {
                    // 999... carried into 1000...: one more decade.
                    digits.insert(0, 1);
                    digits.truncate(precision);
                    exponent += 1;
                    break;
                }
                index -= 1;
                if digits[index] == 9 {
                    digits[index] = 0;
                } else {
                    digits[index] += 1;
                    break;
                }
            }
        }
    }
    while digits.len() < precision {
        digits.push(0);
    }

    (digits.iter().map(|digit| (digit + b'0') as char).collect(), exponent)
}

/// TS `String(value)` for the one interpolation that uses it — the
/// characteristic-length rejection. Rust's `Display` for f64 agrees with
/// JavaScript on ordinary magnitudes; only the non-finite spellings differ.
pub fn js_number_to_string(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }
    if value == f64::INFINITY {
        return "Infinity".to_string();
    }
    if value == f64::NEG_INFINITY {
        return "-Infinity".to_string();
    }
    format!("{value}")
}
