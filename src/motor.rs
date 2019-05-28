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

//! L298 H-bridge motor driver.
//! Inspired by https://github.com/japaric/motor-driver.

extern crate embedded_hal;

use embedded_hal::digital::OutputPin;
use embedded_hal::PwmPin;

/// A full L298 H-bridge motor driver.
pub struct Motor<DIR1, DIR2, PWM>
where
    DIR1: OutputPin,
    DIR2: OutputPin,
    PWM: PwmPin,
{
    dir1: DIR1,
    dir2: DIR2,
    pwm: PWM,
}

impl<DIR1, DIR2, PWM> Motor<DIR1, DIR2, PWM>
where
    DIR1: OutputPin,
    DIR2: OutputPin,
    PWM: PwmPin,
{
    /// Create a new motor driver.
    pub fn new(mut dir1: DIR1, mut dir2: DIR2, mut pwm: PWM) -> Self {
        // initial state: coast
        let _ = dir1.set_low();
        let _ = dir2.set_low();

        pwm.enable();

        Motor { dir1, dir2, pwm }
    }

    /// Makes the motor spin in the forward direction.
    pub fn forward(&mut self) -> &mut Self {
        let _ = self.dir1.set_high();
        let _ = self.dir2.set_low();
        self
    }

    /// Makes the motor spin in the reverse direction.
    pub fn reverse(&mut self) -> &mut Self {
        let _ = self.dir1.set_low();
        let _ = self.dir2.set_high();
        self
    }

    /// Makes the motor coast.
    #[allow(dead_code)] 
    pub fn coast(&mut self) -> &mut Self {
        let _ = self.dir1.set_low();
        let _ = self.dir2.set_low();
        self
    }

    /// Returns the maximum duty cycle for the device.
    pub fn get_max_duty(&mut self) -> PWM::Duty {
        self.pwm.get_max_duty()
    }

    /// Changes the motor duty cycle.
    pub fn duty(&mut self, duty: PWM::Duty) -> &mut Self {
        self.pwm.set_duty(duty);
        self
    }
}
