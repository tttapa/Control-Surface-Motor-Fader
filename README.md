# Control Surface Motor Fader

Motorized fader example code for Arduino using [tttapa/Control-Surface](https://github.com/tttapa/Control-Surface).

Documentation: <https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/>

## Contents

- `Motor-Controller`: Arduino Uno or Nano sketch running PID controllers for up to four motorized faders with touch sensitivity. Optional serial MIDI in/output.
- `MIDI-Controller`: Arduino sketch running [Control Surface](https://github.com/tttapa/Control-Surface) code that communicates with the motor controllers over I²C and allows the faders to be controlled over (USB) MIDI.
- `Python/Tuning.py`: Python script for trying out and comparing different tuning parameters for the motor controllers.

## Demo video

<div align="center">
  <a href="https://www.youtube.com/watch?v=j5vZXO6RVjA"><img src="https://i.ytimg.com/vi_webp/j5vZXO6RVjA/maxresdefault.webp" alt="Arduino - Control Surface - Motorized Fader Demo" width="600"></a><br>
  https://youtu.be/j5vZXO6RVjA
</div>

## Installation

0. Install the Control Surface library ([installation instructions](https://tttapa.github.io/Control-Surface-doc/Doxygen/d8/da8/md_pages_Installation.html))
1. Download the .ZIP file using the green <kbd>Code</kbd>  button on the home page of this repository
2. Extract it to a convenient location on your computer
3. Open the Arduino IDE
4. Use <kbd>Ctrl+O</kbd> or use the `File > Open ...` menu
5. Browse to the `Control-Surface-Motor-Fader-master/Motor-Controller` folder you just extracted and open the `Motor-Controller.ino` file
6. Select the Arduino UNO or Nano in the `Tools > Board` menu and select the correct port
7. Use <kbd>Ctrl+U</kbd> to compile and upload the code to the Arduino

Similarly, you can open and upload the `MIDI-Controller` sketch to a MIDI-capable Arduino such as the Leonardo.
