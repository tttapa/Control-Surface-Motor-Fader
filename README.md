# Control Surface Motor Fader

Motorized fader example code for Arduino using [tttapa/Control-Surface](https://github.com/tttapa/Control-Surface).

Documentation: <https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/>

## Contents

- `Motor-Controller`: Arduino Uno or Nano sketch running PID controllers for up to four motorized faders with touch sensitivity.
- `MIDI-Controller`: Arduino sketch running Control Surface code that communicates with the motor controllers over I²C so the faders can be controlled over MIDI.
- `Python`: Python script for trying out and comparing different tuning parameters for the motor controllers.