# ⚡ ESP32 50Hz Precision Frequency Meter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Arduino](https://img.shields.io/badge/Arduino-compatible-green.svg)](https://www.arduino.cc/)

> 🎯 High-precision grid frequency measurement using I/Q phase-slope method on ESP32

Measure AC grid frequency with **sub-millihertz precision** using affordable ESP32 hardware. Perfect for power quality monitoring, grid stability analysis, and renewable energy integration research.

## ✨ Features

- 📊 **Ultra-high precision**: ~1-2 mHz @ 1s, <0.2 mHz @ 10s (with good SNR)
- 🔌 **Flexible input**: Internal ADC (GPIO36) or external ES8388 codec
- 📡 **MQTT integration**: Real-time metrics publishing
- 🌐 **Web interface**: Built-in HTTP status page
- ⚙️ **FreeRTOS architecture**: Multi-task design for optimal performance
- 🔬 **I/Q demodulation**: Advanced DSP with phase-slope analysis
- 📈 **Quality metrics**: R² correlation and uncertainty estimation

## 🎛️ Technical Specifications

| Parameter | Value |
|-----------|-------|
| Sample Rate | 4 kHz |
| Block Size | 400 samples (0.1s) |
| Analysis Window | 10 seconds |
| Nominal Frequency | 50 Hz |
| Low-pass Filter | 5 Hz (I/Q) |
| Quality Threshold | R² ≥ 0.98 |

## 🛠️ Hardware Requirements

- **ESP32 Development Board** (any variant)
- **Input signal**: Grid voltage via transformer/divider to 0-3.3V
  - Default: GPIO36 (ADC1_CH0)
  - Optional: ES8388 codec for line-in (ESP32-A1S boards)
- **WiFi network** for MQTT and HTTP access

## 📦 Software Requirements

- Arduino IDE 1.8.x or 2.x
- ESP32 Arduino Core **3.x** (tested with 3.3.1)
- Libraries:
  - WiFi (included)
  - WebServer (included)
  - PubSubClient

## 🚀 Quick Start

### 1. Configuration

Edit the following parameters in `50Hz-precisionFrequencyMeter.ino`:

```cpp
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASS     = "YOUR_PASS";
const char* MQTT_HOST     = "192.168.1.10";
const uint16_t MQTT_PORT  = 1883;
```

### 2. Upload

1. Open `50Hz-precisionFrequencyMeter.ino` in Arduino IDE
2. Select your ESP32 board and port
3. Click **Upload**

### 3. Monitor

- **Serial Monitor**: 115200 baud for boot messages
- **Web Interface**: `http://<ESP32-IP>/`
- **MQTT Topics**:
  - `grid/50hzmeter/<device-id>/state` (retained)
  - `grid/50hzmeter/<device-id>/metrics` (real-time)

## 📡 MQTT Output

### State Message (every 60s, retained)
```json
{
  "fw": "arduino-rtos-iq",
  "uptime": 3600,
  "fs_hz": 4000,
  "win_s": 10.0,
  "drops": 0
}
```

### Metrics Message (every 1s)
```json
{
  "ts": 3600.123,
  "f_hz": 50.00012345,
  "sigma_f_mhz": 0.15,
  "r2": 0.9987,
  "window_s": 10.0,
  "drops": 0
}
```

## 🧮 Algorithm Details

The system uses **I/Q demodulation** with **phase-slope estimation**:

1. **Sample**: Acquire 4 kHz ADC samples via I2S
2. **Correlate**: Mix with 50 Hz reference (cos/sin) to extract I/Q components
3. **Filter**: Low-pass filter I/Q at 5 Hz to remove noise
4. **Phase**: Calculate `φ = atan2(Q, I)` for each 0.1s block
5. **Unwrap**: Remove 2π discontinuities
6. **Regress**: Linear fit over 10s window → slope in rad/s
7. **Convert**: `f = slope / (2π)` Hz

Quality is validated using R² correlation coefficient.

## 🔧 Customization

### Adjust precision vs. latency

```cpp
constexpr int WIN_SEC = 10;  // Increase for better precision
```

### Change input mode

```cpp
#define INPUT_MODE INPUT_MODE_ES8388  // For external codec
```

### Tune DSP parameters

```cpp
constexpr float IQ_LP_FC = 5.0f;    // Low-pass cutoff
constexpr float R2_MIN = 0.98f;     // Quality threshold
```

## 📊 Performance

Typical performance with clean AC input (>40dB SNR):

| Window | Frequency Resolution | Update Rate |
|--------|---------------------|-------------|
| 1s     | ~1-2 mHz           | 1 Hz        |
| 10s    | ~0.1-0.2 mHz       | 1 Hz        |

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| High `drops` count | Increase queue sizes or reduce CPU load |
| Low R² values | Check input signal quality and noise levels |
| Compilation errors | Ensure ESP32 Core 3.x is installed |
| No MQTT messages | Verify broker address and network connectivity |

## 📝 License

MIT License - see LICENSE file for details

## 🙏 Credits

Inspired by Andreas Spiess's precision frequency measurement techniques.

## 🔗 Related Projects

- [Grid frequency monitoring](https://github.com/topics/grid-frequency)
- [ESP32 DSP applications](https://github.com/topics/esp32-dsp)

---

**Made with ⚡ by the ESP32 community**
