# SatHack2026

Welcome! Some demo code is provided here for you to get started. Clone (or download) the repository and open it in Arduino IDE. 

# Relevant Functions
Check the file `demo/ACDS.h` to see the base API functions allowing you to read ultrasound data and drive motors.
The file `demo.ino` will run a demo program that uses the ultrasound sensor to avoid obstacles. *Run this first* to test that all the hardware works fine before you start developing your own software.

The demo sketch has recently been refactored to eliminate a
constant 20‑point yaw offset that caused some boards to spin in tight
circles. If you pull the latest version it now ramps speed evenly and
never drives the two motors at different PWM values unless executing a
turn or reverse.

# Notes/Troubleshooting

> I am driving both motors at the same speed, but my RC car is drfiting off to the side!

This happens due to misalignment of the ball wheel on the back. You can fix this by adding a trimming factor to each wheel, for example if you have a top level `motor` function to drive each wheel, you can incorporate a slight imbalance like below.

```c++
void motor(int speed1, int speed2)  
{
  const int trim_1 = 1;
  const int trim_2 = 1.2;
  speed1 *= trim_1;
  speed2 *= trim_2;
  ...
}
```

> I am driving both motors forwards but my car is spinning/going backwards!

Check your connections between the breakout board and motor driver. If this doesn't work, swap pins around. If still confused, reach out to one of the organisers.

> Where can I find more documentation?
Arduino [website](https://docs.arduino.cc/language-reference/).

---