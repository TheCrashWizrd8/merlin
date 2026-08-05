# Sub Dashboard — ESP Telemetry Protocol

The Pi reads telemetry from the ESP32 over a **serial** link (GPIO UART on pins 14/15, or USB).
Lines are ASCII, one message per line.

**GPIO wiring:** see **`docs/PI_GPIO_UART.md`**. Run `sudo bash scripts/setup_pi_uart.sh` once, then use `serial.port: /dev/serial0` in `config/hardware.yaml`.

## ESP → Pi (telemetry)

Text format (prefix `TEL`):

```
TEL battery 12.45
TEL gyro 1.2 -0.5 45.0
TEL depth 3.45
TEL leak 0 0 1 0
TEL ballast 0.5
```

JSON format (alternative):

```json
{"t":"battery","v":12.45}
{"t":"gyro","pitch":1.2,"roll":-0.5,"yaw":45.0}
{"t":"depth","m":3.45}
{"t":"leak","sensors":[false,false,true,false]}
{"t":"ballast","level":0.5}
```

| Field | Unit | Notes |
|-------|------|-------|
| battery | volts | Main pack voltage |
| gyro | degrees | pitch, roll, yaw |
| depth | meters | Depth sensor |
| leak | bool[] | One bool per sensor; `true` = leak detected |
| ballast | 0..1 | Reported fill level |

## Pi → ESP (control)

Sent at ~20 Hz when sub dashboard is running:

```
B <value>\n
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>\n
```

| Command | Range | Description |
|---------|-------|-------------|
| `B` | -1..+1 | Ballast: -1 drain, 0 hold, +1 fill |
| `S2` | -1..+1 each | Aft steering Y and Z |
| `F` | -1..+1 each | Fore fin left and right |
| `X` | -1..+1 | Thruster |

## Dashboard

```bash
python sub_server.py
# or
python inference.py --web --sub --headless
```

Open **http://\<pi-ip\>:8080/sub/**

## Testing without hardware

Use two terminals to exercise the full serial path (fake ESP → `esp_bridge` → dashboard):

```bash
# Terminal 1 — fake ESP (sends TEL lines, prints B / S2 from Pi)
python scripts/simulate_esp_serial.py

# Terminal 2 — sub server on the paired port
python sub_server.py --serial-port /tmp/sub_fake_pi --no-camera --no-xbox
```

Optional: `--leak-alarm` toggles leak sensor 3 every 20s. Use `--hz 5` to match real ESP rate.

`scripts/test_telemetry.py` injects fake data directly into state (no serial) — use that only for UI-only checks.

## Xbox controller (Bluetooth)

The sub server reads Xbox / gamepad input via pygame (SDL2). Works over **USB** or **Bluetooth** once the controller is paired at the OS level.

### 1. Driver (Bluetooth Xbox One / Series)

On Raspberry Pi, Bluetooth Xbox controllers need **xpadneo**:

```bash
bash scripts/setup_xbox_bluetooth.sh
```

Or manually:

```bash
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo && sudo ./install.sh
sudo reboot
```

### 2. Pair the controller

```bash
bluetoothctl
scan on
# Hold Xbox sync button until logo flashes
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

Add your user to the `input` group if `/dev/input/js0` permission is denied:

```bash
sudo usermod -aG input $USER
# log out and back in
```

### 3. Verify detection

```bash
python scripts/test_xbox.py
python scripts/test_xbox.py --watch   # live stick values
```

You should see a device named like `Xbox Wireless Controller`.

### 4. Run sub server with Xbox enabled

```bash
python sub_server.py
# do NOT pass --no-xbox
```

Open **http://\<pi-ip\>:8080/sub/** — badge should show `Xbox — Xbox Wireless Controller`. Sticks map to actuators; LT/RT control ballast.

| Input | Sub axis |
|-------|----------|
| Left stick | Aft steer Y / Z |
| Right stick Y | Thruster |
| Right stick X | Fins (differential) |
| LT / RT | Ballast drain / fill |

Tune deadzone in `config/hardware.yaml` under `xbox:`.

## API endpoints

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/sub/` | Dashboard UI |
| GET | `/sub/api/status` | Connection + leak alarm summary |
| GET | `/sub/api/telemetry` | All sensor readings |
| GET | `/sub/api/telemetry/battery` | Battery voltage |
| GET | `/sub/api/telemetry/gyro` | Pitch / roll / yaw |
| GET | `/sub/api/telemetry/depth` | Depth (m) |
| GET | `/sub/api/telemetry/leaks` | Leak sensor array |
| GET | `/sub/api/telemetry/ballast` | Ballast level + command |
| GET | `/sub/api/serial` | ESP serial log |
| GET | `/sub/api/control` | Control mode + actuators + Xbox state |
| GET | `/sub/api/xbox` | Xbox stick/trigger state |
| POST | `/sub/api/control/ballast` | `{"action":"fill"\|"drain"\|"neutral"}` or `{"value":0.5}` |
| POST | `/sub/api/control` | `{"mode":"xbox"\|"manual"\|"auto"}` |
| POST | `/sub/api/control/actuators` | Manual actuator overrides |
| POST | `/sub/api/serial` | Send raw line to ESP |
| GET | `/video_feed` | USB camera MJPEG stream |
