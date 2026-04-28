# Diff Assist Protocol

This repository keeps the original Ackermann steering logic as the primary control path.
Rear-wheel differential assist is added as an opt-in enhancement.

## Linux-side behavior

- `diffEnable = 0`: keep the original behavior
  - send single speed packet: `0x50`
  - send servo packet: `0x60`
- `diffEnable = 1` and `abs(steer_cmd) > diffThreshold`
  - send differential speed packet: `0x51`
  - send servo packet: `0x60`

## Differential speed packet

Packet layout:

```text
[0xAA, 0x55, 0x0B, 0x51]
+ left_cmd(2 bytes, signed big-endian)
+ right_cmd(2 bytes, signed big-endian)
+ 0x00, 0x00
+ checksum(1 byte)
```

Checksum rule:

- Sum of bytes `0..9`, low 8 bits kept in byte `10`

Speed unit:

- same unit as the existing speed packet in `src/control.cpp`
- current Linux code uses `m/s -> mm/s`, then clamps to `[-1200, 1200]`

## Assist formula

The first-pass tuning logic is:

```text
assist = min(
    diffLimit,
    (abs(steer_cmd) - diffThreshold) * diffGain,
    abs(speed_cmd) * 0.4
)
```

Then:

```text
left_cmd  = speed_cmd - signed_assist
right_cmd = speed_cmd + signed_assist
```

Notes:

- `signed_assist` follows `steer_cmd` direction
- `diffSign` can be set to `-1` if the vehicle rotates the wrong way

## Current params

- `params/diffEnable`
- `params/diffGain`
- `params/diffLimit`
- `params/diffThreshold`
- `params/diffSign`

## Firmware integration note

The current workspace does not include the exact STM32 firmware that already understands the Linux-side `0x50 / 0x60` packets.
Because of that, the new `0x51` packet is defined here as the reference extension for the OpenCTR side to implement.
