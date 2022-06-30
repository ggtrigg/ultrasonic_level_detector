# Ultrasonic Level Detector
This is a small project utilising a Raspberry Pi Pico and a RCWL-1601 (or HC-SR04) ultrasonic transducer.
It simulates a scenario where a container starts filling when it approaches empty and stops filling as it
approaches being full.
This hysteresis is performed by measuring the ultrasonic echo pulse distance and turning on the onboard LED
(which in real life could turn on a motor etc.) when the distance moves beyond the "empty" threashold, and then
turning the LED off when the distance becomes closer than the "full" threshold.

This was mainly an exercise in learning embedded Rust using a possible real world situation for stimulus.
![image](https://user-images.githubusercontent.com/32510770/176601161-07d6b935-24ab-4a65-b27e-d1de422f76e6.png)
