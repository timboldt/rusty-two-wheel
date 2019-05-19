//  Copyright 2019 Google LLC
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

//! A class for managing a controlled-speed wheel. It keeps track of velocity
//! and total distance travelled. It also has a PID controller for computing
//! the amount of power to send to the motor to maintain the target speed.

/// A controlled-speed wheel.
/// NOTE: For the hardware I'm using, a 50% duty cycle produces about 5000 ticks
/// per second.
pub struct Wheel {
    // Total travel in ticks.
    // Note: The tick rate is < 500/s, so 32-bits is enough for more than
    //       1 day of travel.
    odometer: i32,
    // Velocity of wheel in ticks per second.
    // TODO: The current velocity calculation is naive and probably very
    // noisy for high update rates.
    velocity: i32,
    // Most recent reading from the 16-bit encoder.
    last_reading: u16,
    // Elapsed time per update, in ms.
    elapsed_ms: i32,
}

impl Wheel {
    pub fn new(current_reading: u16, loop_ms: u32) -> Self {
        Wheel {
            odometer: 0,
            velocity: 0,
            last_reading: current_reading,
            elapsed_ms: loop_ms as i32,
        }
    }

    pub fn update(&mut self, current_reading: u16) {
        let mut delta_ticks = current_reading.wrapping_sub(self.last_reading) as i32;

        // Assuming the wheel cannot move anywhere near 50% of the
        // encoder's total range, large numbers must have been due
        // to negative travel.
        if delta_ticks >= 32768 {
            delta_ticks -= 65536;
        }

        self.odometer += delta_ticks;
        // Smooth out the velocity.
        // TODO: Make this configurable, since it makes less sense
        // at slower update rates.
        let smoothing = 4;
        let mut raw_velocity = delta_ticks * 1000 / self.elapsed_ms;
        self.velocity = (self.velocity * (smoothing - 1) + raw_velocity) / smoothing;
        self.last_reading = current_reading;
    }

    pub fn odometer(&self) -> i32 {
        self.odometer // TODO: maybe output in real units like mm or meters.
    }

    pub fn velocity(&self) -> i32 {
        self.velocity // TODO: maybe output in real units like mm or meters.
    }
}
