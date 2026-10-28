# Omron G5LE-14 — leak / fault LED relay

The **G5LE-14** is a **bare 5-pin SPDT PCB relay** (coil + COM/NC/NO). It is **not** safe to drive the coil directly from a Pi GPIO (≈79 mA @ 5 V on the **G5LE-14-DC5** coil).

In this project you almost certainly have either:

1. A **relay module** (blue board) that mounts a G5LE (or clone) and exposes **IN / RCIN**, **VCC**, **GND**, and screw terminals **COM / NC / NO**, or  
2. The **bare relay** plus your own transistor / flyback diode driver.

Software is the same: Pi **BCM GPIO 18** → module **signal input**; firmware does **not** drive the relay (see `src/fault_indicator.py`).

---

## G5LE-14 relay pins (bottom view, Omron)

SPDT — one common, one normally closed, one normally open:

| Pin | Function |
|-----|----------|
| 1 | Coil **+** (e.g. +5 V for DC5 part) |
| 2 | Coil **−** (switch to GND via driver) |
| 3 | **COM** |
| 4 | **NC** (connected to COM when coil **off**) |
| 5 | **NO** (connected to COM when coil **on**) |

Coil ratings depend on suffix (**G5LE-14-DC5** = 5 V DC, **DC12**, **DC24**, etc.). Use the voltage printed on the relay.

**LEDs on NC (this sub):** run lamp **+** through **COM ↔ NC** so the lamp is **on** when the coil is **off** and **off** when the coil is **on**.

Example (low-voltage LED string):

```text
  LED supply +  ──►  LED anode string  ──►  LED cathode / return
                                              │
  LED return    ◄──  COM (pin 3)
  (or + to COM)      NC (pin 4)  when coil off → LEDs on
```

Exact polarity depends on your LED wiring; keep LED supply current within relay contact rating (check datasheet; NC is often rated lower than NO on some G5LE variants).

---

## Typical relay **module** (IN / RCIN / VCC / GND)

Most Pi-friendly boards:

| Module terminal | Connect to |
|-----------------|------------|
| **VCC** / **DC+** | **5 V** (Pi pin 2 or 4) — **not** 3.3 V for a 5 V coil module |
| **GND** / **DC−** | Pi **GND** (must be common with signal) |
| **IN** / **RCIN** / **SIG** | Pi **GPIO 18** (BCM), physical header **pin 12** |
| **COM** | LED circuit common |
| **NC** | LED circuit (your LEDs are on **NC**, not NO) |
| **NO** | Unused for this annunciator |

Many modules are **active-low** on IN: **LOW** = relay **energized**, **HIGH** = released. Others are active-high. That is why `config/hardware.yaml` has **`idle_rcin_level`** and **`fault_rcin_level`** instead of assuming one polarity.

---

## Desired behavior (NC LEDs)

| Condition | Coil | NC contact | LEDs |
|-----------|------|------------|------|
| Normal (no leak) | **Energized** | Open | **Off** |
| Leak / ESP fault | **Released** | Closed | **On** |

---

## Software implementation (Pi)

| Piece | Role |
|-------|------|
| ESP `sub_rc` | Reads leak GPIO; sends `TEL leak …` and `TEL fault …` |
| `src/esp_bridge.py` | Parses TEL → `notify_tel_leak()` / `notify_tel_fault()` |
| `src/fault_indicator.py` | Drives GPIO 18 via **`pinctrl`** (Pi 5); holds level until next TEL |
| `config/hardware.yaml` | `fault_indicator:` block |

Current settings (tune if LEDs are inverted):

```yaml
fault_indicator:
  enabled: true
  gpio: 18
  backend: pinctrl
  idle_rcin_level: 1    # no fault → coil on → NC open → LEDs off
  fault_rcin_level: 0   # fault → coil off → NC closed → LEDs on
```

Restart **`~/sub`** after edits. Boot line example:

`[fault_indicator] BCM18 (pinctrl, idle=1 fault=0, drive_ok=True)`

Fault only updates when ESP sends **`TEL leak`** or **`TEL fault`** (not from the dashboard).

---

## Bench test (Pi 5 — use `pinctrl`, not `gpioset`)

```bash
pinctrl get 18
pinctrl set 18 op dh    # try HIGH — see if NC LEDs go off
pinctrl set 18 op dl    # try LOW  — see if NC LEDs turn on
```

If **dh** and **dl** do the opposite of what you want, swap **`idle_rcin_level`** and **`fault_rcin_level`** in `hardware.yaml`.

If **`pinctrl` changes `get 18` but LEDs never move**, check:

- **VCC** on the module (5 V present?)  
- **GND** shared Pi ↔ module  
- Signal on **pin 12** (BCM 18), not a different header pin  
- LED power through **COM + NC**, not NO  

---

## References

- [Omron G5LE datasheet (PDF)](https://omronfs.omron.com/en_US/ecb/products/pdf/en-g5le.pdf)  
- Pi header GPIO: [WIRING.md](WIRING.md) (Pi GPIO section)
