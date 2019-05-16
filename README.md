## Rusty Two Wheel

### Disclaimer

This is not an official Google product (experimental or otherwise).

### Introduction

This project is a work-in-progress implementation of a two-wheel balancing
robot, using an STM32F103 "Blue Pill", an L298N motor controller, a MPU9250
IMU, and a pair of geared DC motors with hall effect encoders.

I've previously implemented something similar on Arduino, STM32Duino, and
also using plain C and STM32CubeMX. As such, I though it would be interesting
to see what I could accomplish in Rust.

### TODO

This is very much a weekend hacking project. I have snippets of code that
can talk to all of the attached sensors and motors, but there are no complete
systems working as of May 2019.
