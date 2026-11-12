# NMEA2000_Victron: Cerbo GX / MultiPlus-II

Encode/decode helpers missing from this project's installed NMEA2000 library
(4.22.1). 
No application handlers, gauges, settings, source selections or transmissions
are changed by this library. 

## Coverage

| PGN | Encode/decode helpers | Purpose |
| --- | --- | --- |
| 61184 / 126720 | `Set/ParseN2kVictronVreg` | Victron registers, single frame / fast packet |
| 65005 | `Set/ParseN2kUtilityAcEnergy` | Export/import energy in kWh |
| 65006, 65009, 65012, 65015 | `Set/ParseN2kUtilityAcReactivePower` | Reactive power and power factor: phase C, B, A, total |
| 65007, 65010, 65013, 65016 | `Set/ParseN2kUtilityAcPower` | Real/apparent power: phase C, B, A, total |
| 65008, 65011, 65014, 65017 | `Set/ParseN2kUtilityAcBasic` | Voltage/current/frequency: phase C, B, A, average |
| 127503 / 127504 | `Set/ParseN2kLegacyAcStatus` | Legacy AC input/output status |
| 127509 | `Set/ParseN2kPGN127509` | Legacy inverter status |
| 127744..127746 | `Set/ParseN2kAcPowerCurrent` | AC current and signed power, phases A/B/C |
| 127747..127749 | `Set/ParseN2kAcVoltageFrequency` | AC voltage and frequency, phases A/B/C |

The phase/aggregate AC codecs cover the related message families, not a promise
that every Cerbo firmware/device broadcasts every PGN. Victron's product table
specifically documents 65013/65014 for VE.Bus L1 power/basic measurements and
127744 for some chargers; older status messages can be disabled or superseded.
Observe the actual bus to determine the available values and cadence.

Already in the normal library, so intentionally **not duplicated**:
127501/127502 (binary status/control), 127505 (tanks), 127506 (DC status),
127507 (charger status), 127508 (battery status), 127513 (battery configuration),
127750 (converter status), plus ISO/device information and group functions.

Setters build `tN2kMsg` objects, not bus frames, and never call `SendMsg`.
Parsers consume complete messages reassembled by the NMEA2000 library.
All helpers return `bool`; callers must check it. Failed calls leave outputs
unchanged. Setters reject non-finite/out-of-range measurements; use
`N2kDoubleNA` for unavailable values. Parsers reject wrong PGNs, truncated
payloads, and non-Victron proprietary headers. Numeric unavailable/error/reserved
encodings are mapped to `N2kDoubleNA` in the AC measurements.

J1939 power is **offset binary** (`raw - 2000000000`), not two's-complement.
J1939 frequency uses 1/128 Hz; its voltage/current fields use whole V/A.
Modern 127744..127746 power is signed two's-complement, current is 0.1 A.
Measurements otherwise use V, A, Hz, W, VA, var and kWh.

## Victron registers and control

Register helpers implement VREQ (`0x0001`), VACK (`0x0002`), and register values.
The generic payload API supports all register IDs, including product-specific
registers and fast-packet values up to 219 bytes. Single-frame values are
zero-padded to four data bytes, so the original unpadded length cannot be
recovered. PGN 126720 uses only its actual data length.

`N2kVictronRegister` names mode/state, remote-control flags, active input,
current-limit and consumed-Ah registers. `ParseN2kVictronRegisterValue` returns
raw little-endian values up to 32 bits:

- Mode `0x0200`: 1 charger-only, 2 inverter-only, 3 on, 4 off, 5 eco
  (only on devices that support eco).
- State `0x0201`: Victron device-state codes, **not** `N2kInverterState` from
  PGN 127509. For example, Victron state 9 means inverting.
- Remote control `0x0202`: feature bitmask, not a command to enable everything.
- Current-limit registers: unsigned 0.1 A; `0xffff` means unavailable/internal
  as defined for the particular register.
- Consumed Ah `0xeeff`: signed 32-bit two's-complement, scale 0.1 Ah.

For application control, first discover the **inverter virtual device**
(Victron function 153) by ISO NAME and resolve its current address. Do not
send VREG control to the AC-input virtual device (function 154).
Victron's use of these function codes differs from the generic NMEA list.
NMEA2000-out must be enabled on the GX device.

Before writing, read the current mode, remote-control mask and current-limit
min/max, preserve unrelated mask bits, and follow the remote-control
negotiation in Victron's register document. A Digital Multi Control or other
owner can refuse changes. Never replay saved control values at startup or
continually send commands. Check the device's broadcast readback and VACK
errors (e.g. `0xc001` control owned elsewhere); an accepted bus transmission
is not confirmation that the MultiPlus applied the command.

Example of **constructing**, not transmitting, a read request:

```cpp
#include <N2kVictron.h>

tN2kMsg request;
bool ok = SetN2kVictronRegisterRequest(
    request, inverterAddress,
    static_cast<uint16_t>(N2kVictronRegister::DeviceMode));
// Handle !ok; transmit only when the application's control logic allows it.
```

Convenience constructors `SetN2kVictronDeviceMode`,
`SetN2kVictronRemoteControl` and `SetN2kVictronCurrentLimit` reject broadcast
and null destinations. `SetN2kVictronCurrentLimit` takes amperes and input
1 or 2; it validates the wire range, **not** the connected product's limits.
Generic register constructors also allow broadcasts for announcements and
reads; they cannot determine whether a raw payload is a command.

## Later application integration

Keep data keyed by source/ISO NAME, PGN and instance/phase. A Cerbo uses
multiple virtual device addresses, not one address for the entire system.
Both AC-input and AC-output virtual devices can emit the same PGNs.

Before opening the bus, register any additional received PGNs using
`ExtendSingleFrameMessages` (J1939 AC, 127744..127749 as required) and
`ExtendFastPacketMessages` if needed. Include transmitted PGNs in
`ExtendTransmitMessages` when an application actually sends these messages.
PGNs 61184/126720 have proprietary framing already recognised by NMEA2000;
the helper does not change transport configuration or application filters.
Do not represent a fast packet as a single raw CAN frame.

## Validation

The native tests use the real installed `tN2kMsg`, not mocks. From the project
root (adjust `N2K` to the installed library directory if necessary):

```sh
N2K=.pio/libdeps/esp32-s3-devkitc-1/NMEA2000-library/src
g++ -std=c++11 -Wall -Wextra -fsanitize=address,undefined \
  -fno-omit-frame-pointer -Ilib/NMEA2000_Victron -I"$N2K" \
  lib/NMEA2000_Victron/tests/test_victron.cpp \
  lib/NMEA2000_Victron/N2kVictron.cpp \
  "$N2K/N2kMsg.cpp" "$N2K/N2kStream.cpp" "$N2K/N2kTimer.cpp" \
  -o /tmp/cerbo-custom-tests
UBSAN_OPTIONS=halt_on_error=1 /tmp/cerbo-custom-tests
```

Tests cover byte-level fixtures, all supported phases/groups, NA and bounds,
negative/offset power, malformed messages and maximum VREG fast packets.
Hardware interoperability still needs validation with the actual Cerbo firmware.

## Protocol sources

- [Cerbo GX NMEA 2000 integration](https://www.victronenergy.com/media/pg/Cerbo_GX/en/marine-mfd-integration-by-nmea-2000.html)
- [Victron VE.Can registers](https://www.victronenergy.com/upload/documents/VE.Can-registers-public.pdf)
- [Victron data communication whitepaper](https://www.victronenergy.com/upload/documents/Whitepaper-Data-communication-with-Victron-Energy-products_EN.pdf)
- [CANboat public PGN field definitions](https://github.com/canboat/canboat/blob/master/docs/canboat.json)
