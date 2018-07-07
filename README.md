# arduino-with-mavlink
Tutorial that explains connection with a quadrocopter based on APM 2.6 with an Arduino ESP32 board. Project is made like a part of the DroneTeam project.

## Getting Started

To start creating briged connection, you have to download the following libraries and software.

### Software

This is a mandatory software that you have to include into the project to compile it successfully. 

* [Arduino](https://www.arduino.cc/en/Main/Software) - Arduino IDE
* [ESP32](https://github.com/espressif/arduino-esp32) - Esp32 arduino library
* [Mavlink](https://github.com/mavlink/mavlink) - Mavlink core library

### Sources

Below there are some links with really helpful knowledge. Mostly you will find only the official mavlink documentation which isn't sufficient and clear for begginers.

* [Basics](https://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf) - A great document with explanation of the elementary issues of mavlink messages and issues.

* [Mavlink common](http://mavlink.org/messages/common) - Official mavlink's list with the all commands. The most important are numbers that stands for the appropriate message.

* [Mavlink's code](https://discuss.ardupilot.org/t/mavlink-step-by-step/9629) - This article contains step by step entry for coding in mavlink. Highly recommended to read before starting to code.

* [Mavlink and arduino](https://discuss.ardupilot.org/t/mavlink-step-by-step/25566) - This is a working example of professional usage mavlink protocol in the Arduino environment. Requires a lot of time to understand everything at all.

```
MIT License

Copyright (c) 2018 Tomasz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
