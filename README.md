# Sound-Following Robot: Embedded DSP and Motor Control

Recovered MSP432 firmware stages from a two-person EEC 10 project at UC Davis. The project combined microphone acquisition, digital filtering, sound detection, and H-bridge motor control for a small autonomous robot.

## Architecture

```text
left/right microphones
        |
        v
MSP432 dual ADC sampling
        |
        v
high-pass (200 Hz) -> low-pass (3,000 Hz)
        |
        v
segmented amplitude / sound decision
        |
        v
direction state -> Timer_A PWM -> H-bridge motors
```

## Recovered source stages

| File | What it demonstrates |
| --- | --- |
| `firmware/sound_motion.c` | Dual-channel sampling, DC-removing high-pass filtering, segmented peak averaging, sound/quiet decisions, and PWM motor commands |
| `firmware/bandpass_filters.c` | 10 kHz dual-channel ADC acquisition with cascaded 200 Hz high-pass and 3,000 Hz low-pass filters |

## Hardware and tools

- TI MSP432P4xx microcontroller and DriverLib.
- Code Composer Studio.
- Two analog microphone channels on ADC inputs A14/A15.
- Timer_A sampling interrupts.
- Timer_A PWM outputs driving a dual H-bridge motor stage.

## Project behavior

The completed course project compared stereo sound levels to distinguish front, left, right, rear, and silence states, then coordinated sampling pauses and motor-settle delays to reduce electrical and mechanical interference.

The local archive contained the filtering and motor-control stages, but not a clean final merged Code Composer Studio project. For accuracy, this repository preserves those recovered stages instead of presenting them as a directly flashable final build.

## Build notes

The source expects TI DriverLib plus the course support modules `Clock.h` and `SysTick.h`. Those external support files and generated Code Composer Studio build artifacts are not included.

## Collaboration note

This was a two-person final project. The repository documents recovered firmware work without claiming sole authorship of the full robot.
