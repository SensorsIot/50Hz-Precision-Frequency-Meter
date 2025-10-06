# ESP32 50 Hz Frequency Meter — Functional Specification (FSD)

**Revision:** v1.0  
**Date:** 2025‑10‑06  
**Owner:** Andreas Spiess  
**Target HW:** ESP32‑A1S (ES8388 audio codec) or standard ESP32 with I2S ADC  
**License:** MIT

---

## 1. Purpose & Scope
Design and implement a mains‑frequency measurement instrument for 50 Hz grids using an ESP32 with an I2S ADC (ES8388). The device samples an isolated, scaled mains waveform, derives instantaneous phase, and computes frequency from the phase slope with robust statistics. It publishes results via MQTT and stores them in InfluxDB; a small HTTP UI provides status and configuration.

**Primary KPIs**
- **Short‑term stability (1 s window):** ≤ 2 mHz RMS (typ.)
- **Mid‑term stability (10 s window):** ≤ 0.2 mHz RMS (typ.)
- **Absolute accuracy:** limited by sampling clock; with ±1 ppm clock → ±0.00005 Hz at 50 Hz. Optionally TCXO/GPSDO improves to µHz‑class.

**Non‑Goals**
- Power‑quality (THD, flicker) analysis beyond basic SNR and harmonic indicators.
- Direct connection to mains (always galvanic isolation).

---

## 2. System Overview
**Signal chain:** 230 V AC → safety isolation transformer → attenuation/divider → anti‑alias RC → ES8388 line‑in → I2S @ 4 kS/s → DSP (band‑limit, analytic signal, phase unwrap) → robust linear regression → frequency → MQTT/HTTP/InfluxDB.

**Clocking:** Device uses board audio clock (e.g., 12.288 MHz → 48 kHz base) divided to 4 kS/s. Optional TCXO or GPS‑disciplined clock recommended for high absolute accuracy.

**Outputs:** Frequency [Hz], rolling σ, SNR, fit R², health/alarms.

---

## 3. Safety & Compliance
- Use a **mains‑rated isolation transformer** (≥ 1 kV isolation) with primary fuse and proper creepage/clearance.
- Enclose primary wiring; earth chassis if metal. 
- Never expose low‑voltage circuits to the primary side. 
- Follow local electrical codes.

---

## 4. Hardware Requirements
### 4.1 Components
- ESP32‑A1S devboard (with **ES8388** codec), or ESP32 + external I2S ADC (≥ 16 bit).  
- Isolation transformer, e.g., **230 → 6 V, 1–2 VA** (low‑distortion type preferred).  
- Primary fuse (T80 mA–T160 mA), MOV + RC snubber per transformer datasheet (optional but recommended).  
- Secondary divider to ~±1 Vpeak at codec input (example: **R1 = 22 kΩ, R2 = 1.8 kΩ** to ground, tweak per transformer output).  
- Anti‑alias RC: **Raa = 330 Ω, Caa = 2.2 µF** (fc ≈ 220 Hz).  
- Input protection: series 330 Ω + clamp diodes to codec rails if needed; keep input level within ES8388 line‑in limits.  
- Optional clock upgrade: **TCXO** at audio master frequency (e.g., 12.288 MHz ±0.5 ppm) or GPSDO→PLL.

### 4.2 Electrical/Analog Specs
- Input after transformer: sine‑like, nominal 6 Vrms (no‑load may rise).  
- Target ADC full‑scale utilization: 50–80 % of FS on peaks.  
- Effective bandwidth at ADC input: ~0–200 Hz (≥ 40 dB attenuation at 1000 Hz).  
- THD not critical for frequency; harmonic energy tolerated by DSP.

### 4.3 Power & Housing
- 5 V USB supply for ESP32.  
- Transformer powered from mains; keep primary enclosed.  
- Provide strain relief and labeling.

---

## 5. Firmware Requirements (ESP‑IDF or Arduino‑ESP32)
### 5.1 Tasks & Concurrency
- **I2S Capture Task:** Reads interleaved PCM blocks (mono) at **fs = 4000 Hz**, 16‑bit. Double‑buffered, ≥ 0.5 s of buffer.  
- **DSP Task:** Processes 1.0–10.0 s windows; overlaps allowed (e.g., 1 s hop).  
- **Publisher Task:** MQTT + local HTTP server; rate‑limits to configured publish period (e.g., 1 Hz).

### 5.2 Configuration (persisted JSON/TOML)
- Sampling: `fs`, window length `T_win` (1…20 s), hop `T_hop` (0.5…5 s).  
- Filters: LPF cutoff (default 120 Hz), analytic method (`hilbert`|`IQ`).  
- Robust stats: outlier filter (`hampel` with window=21, k=3), regression type (`ols`|`theil_sen`).  
- MQTT: broker URL, topic base, QoS, retain, credentials.  
- InfluxDB (v1 or v2): enable, URL, org/bucket/db, auth.  
- HTTP UI: enable, port, CORS.  
- Clock correction (optional): `ppm_correction` float.  
- Alarms: thresholds for freq deviation, R² min, SNR min.

### 5.3 Telemetry & Logging
- Log levels: `error|warn|info|debug|trace`.  
- Health: boot time, uptime, dropped buffers, CPU load, heap, last publish status.

---

## 6. DSP & Algorithms
### 6.1 Pre‑Processing
1. **Detrend & scale** audio block to float [−1,1].  
2. **Band‑limit** with FIR LPF, fc ≈ 120 Hz (e.g., 129 taps, Hamming).  
3. Optional **notch** at 150 Hz if third harmonic pollution is high (usually unnecessary).

### 6.2 Analytic Signal
Two interchangeable methods (configurable):
- **Hilbert transform:** `xa = x + j * hilbert(x)` (FIR Hilbert; odd‑symmetric, e.g., 129 taps).  
- **Digital I/Q:** Multiply x[n] by numerically generated sin/cos(2π·50 Hz·n/fs) → low‑pass (fc ≈ 5–10 Hz) to remove 100 Hz terms. Prefer Hilbert for simplicity and determinism.

Compute instantaneous **phase**: `phi[n] = atan2(Im(xa[n]), Re(xa[n]))`, then **unwrap**.

### 6.3 Frequency Estimation (Phase‑Slope)
On each analysis window of duration `T_win` seconds with N samples and timestamps `t[n]`:
- Fit `phi[n] ≈ a * t[n] + b` using robust linear regression (default **OLS** with Hampel prefilter; option **Theil–Sen**).  
- Frequency:  
  \[ f = \frac{a}{2\pi} \quad [\text{Hz}] \]
- Uncertainty: use residual std. dev. σφ and time variance to estimate σf.  
- Quality metric: **R²** of fit; publish `R2` and `sigma_f`.

### 6.4 Outlier Handling & Quality Gates
- **Hampel filter** on phase increments Δφ to reject sudden steps due to spikes; window = 21, k = 3.  
- Discard windows with: R² < 0.98, `sigma_f` > configured limit, or SNR < threshold.  
- **SNR estimate:** ratio of energy within ±5 Hz of 50 Hz vs remainder in 0–200 Hz band (pre‑bandlimited buffer).

### 6.5 Timing & Clock Considerations
- Timestamp samples by sample index; **no RTC needed** for frequency.  
- Optional `ppm_correction` applied to `fs` in calculations.  
- With TCXO (±0.5 ppm), absolute error at 50 Hz ≤ 0.000025 Hz.

---

## 7. Interfaces
### 7.1 MQTT Topics (example)
Base: `grid/50hzmeter/<device_id>/`
- `state` (JSON, retained): firmware, config hash, uptime, last_ok.  
- `metrics` (JSON, 1 Hz):  
  ```json
  {
    "ts": 1738848000.123,
    "f_hz": 49.99982,
    "sigma_f_mhz": 0.12,
    "r2": 0.996,
    "snr_db": 38.4,
    "window_s": 10.0,
    "hop_s": 1.0,
    "fs_hz": 4000,
    "drops": 0
  }
  ```
- `alarm` (JSON, on change): `{ "type": "low_r2" | "low_snr" | "freq_deviation", "value": ... }`

### 7.2 InfluxDB Schema (line protocol)
- **Measurement:** `gridfreq`
- **Tags:** `device_id`, `site`, `grid` (e.g., "UCTE")
- **Fields:** `f_hz` (float), `sigma_f_mhz` (float), `r2` (float), `snr_db` (float), `window_s` (float), `drops` (int)
- **Timestamp:** publish in ns or ms per Influx config

Example:  
```
gridfreq,device_id=esp32-a1s-01,site=lab,grid=UCTE f_hz=49.99982,sigma_f_mhz=0.12,r2=0.996,snr_db=38.4,window_s=10.0,drops=0 1738848000123456789
```

### 7.3 HTTP UI (minimal)
- `/` Status page: live value, sparkline, R², SNR, window/ hop.  
- `/config` GET/POST JSON: full configuration with validation.  
- `/health` JSON: uptime, heap, CPU, last error, drops.

---

## 8. Calibration & Validation
1. **Level check:** Verify ADC input ≈ 0.7 FS on peaks with nominal mains.  
2. **Clock calibration (optional):** Measure a known tone (function generator) at 50.00000 Hz; solve for `ppm_correction` to minimize mean error over ≥ 60 s.  
3. **Cross‑check:** Compare with a trusted source (e.g., mainsfrequency.com EU feed) within expected geographic correlation.  
4. **Stability test:** Allan deviation over 1–60 s; confirm σ scales ~1/√τ.

---

## 9. Performance Targets & Acceptance Criteria
- Startup to first valid frequency: ≤ 3 s (with T_win = 1 s).  
- No audio buffer overruns during 24 h run.  
- With decent SNR (> 30 dB), report σf ≤ 2 mHz @ 1 s, ≤ 0.2 mHz @ 10 s.  
- MQTT publish success rate ≥ 99.9 % over LAN.  
- Configuration persists across reboot; invalid values rejected with error.

---

## 10. Test Plan (Essentials)
- **Unit tests (DSP):** FIR response, Hilbert phase accuracy on synthetic data, regression slope recovery on chirps ±0.02 Hz.  
- **Integration tests:** Record 60 s of ADC data; offline MATLAB/Python reference vs device output Δf ≤ 0.1 mHz (same windowing).  
- **Fault injection:** Add impulsive noise bursts; verify Hampel removes outliers and quality gates behave.  
- **Throughput:** Stress I2S at 8 kS/s to verify headroom; then run at 4 kS/s.

---

## 11. Future Enhancements
- **Clock discipline:** GPSDO → Si5351/PLL for absolute µHz accuracy.  
- **Multi‑band PQ:** THD, harmonic amplitudes (FFT side‑task).  
- **NTP‑stamped frames:** For multi‑site phase comparison (requires known time base).  
- **Web UI charts:** rolling Allan deviation, histograms.

---

## 12. Pseudocode (Reference)
```pseudo
init():
  load_config()
  i2s_init(fs=4000, mono16)
  dsp_init_fir_lpf(fc=120Hz, taps=129)
  hilbert_init(taps=129)  # or IQ path
  mqtt_init()
  http_init()

main_loop():
  block = i2s_read_block(N = fs * T_hop)  # e.g., 1s hop → N=4000
  ring_buffer.push(block)
  if ring_buffer.length >= fs * T_win:
    x = ring_buffer.last(fs * T_win)
    x = lpf_fir(x)
    xa = hilbert(x)                     # complex analytic
    phi = unwrap(atan2(imag(xa), real(xa)))
    t = [0..len(phi)-1] / fs
    phi = hampel_filter(phi, w=21, k=3)
    (a,b,R2) = linear_regression(t, phi)
    f = a / (2*pi)
    sigma_f = estimate_sigma_f(phi, t, a)
    snr = estimate_snr(x, fs)
    if R2 >= R2_min and snr >= snr_min:
      publish_mqtt(f, sigma_f, R2, snr, ...)
      write_influx(f, sigma_f, R2, snr, ...)
    update_http_cache(...)
```

---

## 13. Bill of Materials (example)
- ESP32‑A1S devkit (ES8388)
- Isolation transformer 230→6 V, 1–2 VA
- Fuse T100 mA, fuse holder; MOV 275 VAC; RC snubber (e.g., 100 Ω + 100 nF X2)
- Resistors: 22 kΩ, 1.8 kΩ, 330 Ω (1%)
- Capacitors: 2.2 µF film/electrolytic (≥ 16 V), assorted 100 nF decouplers
- Enclosure, terminal block, wire, perfboard/PCB
- Optional: 12.288 MHz **TCXO** (±0.5 ppm) compatible with board clocking

---

## 14. Risks & Mitigations
- **Transformer saturation/line variance:** choose adequate VA rating; keep input level margin; use divider to limit ADC peaks.  
- **Clock drift:** add `ppm_correction`; upgrade to TCXO/GPSDO.  
- **Noise/harmonics:** sufficient band‑limit and robust phase slope mitigate; optional shielded wiring.  
- **Task starvation:** use pinned FreeRTOS tasks with priorities; monitor drops.

---

## 15. Deliverables
- Firmware source (ESP‑IDF or Arduino) with CI build.  
- Config defaults file; sample MQTT/Influx dashboards.  
- Hardware wiring diagram and safety notes.  
- README with quick‑start and calibration steps.

---

## 16. Notes / how to wire quickly

- **Internal ADC mode (default):** Feed your transformer/divider output into **GPIO36 (ADC1_CH0)**. Keep it within ~0.1–2.7 V. Add a small **RC (≈330 Ω + 2.2 µF)** to band-limit around 200 Hz.
- **Publishing:** MQTT topics are `grid/50hzmeter/<device_id>/state` and `/metrics` (JSON).
- **Quality:** `r2` is the regression fit quality; if it falls below ~0.98 you’ll see noisier readings.
- **Window:** ~10 s by default (100 blocks × 0.1 s). Change `WIN_SEC` to 1–20 s based on desired stability.

### Switching to ES8388 line-in (ESP32-A1S)

1. Change `#define INPUT_MODE` to `INPUT_MODE_ES8388`.
2. Fill `initES8388LineIn()` with your known-good codec register setup (LINE IN → ADC → I²S, left-justified or I²S standard, 16-bit, 4 kHz or 8 kHz with decimation).
3. Set `i2s_pin_config_t` to your board’s BCK/WS/SDIN pins (the example uses common A1S pins).

If you want, I can drop a ready-to-go **ES8388 init** tailored to your A1S dev board and even add **InfluxDB line-protocol** posting and a **JSON config** endpoint—all within this sketch.
