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

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
// extern crate cortex_m_semihosting as sh;
extern crate jlink_rtt;
// extern crate madgwick;
extern crate mpu9250_i2c;
#[macro_use(block)]
extern crate nb;
extern crate panic_rtt;
extern crate pid;
extern crate stm32f1xx_hal as hal;

use crate::hal::delay::Delay;
use crate::hal::i2c::BlockingI2c;
use crate::hal::prelude::*;
use crate::rt::{entry, ExceptionFrame};

use core::fmt::Write;
//use madgwick::{F32x3, Marg};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use pid::Pid;

#[entry]
fn main() -> ! {
    let mut output = jlink_rtt::Output::new();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Set up clock tree for maximum performance.
    // * 8MHz external crystal.
    // * System clock at its maximum value of 72MHz.
    // * APB1 peripheral clock restricted to its maximum value of 36MHz.
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // User LED.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // SYST timer.
    //let mut timer = hal::timer::Timer::syst(cp.SYST, 1000.hz(), clocks);

    // MPU 9250 IMU
    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    let mut i2c = BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        hal::i2c::Mode::Fast {
            frequency: 400_000,
            duty_cycle: hal::i2c::DutyCycle::Ratio2to1,
        },
        clocks,
        &mut rcc.apb1,
        1_000,
        2,
        1_000,
        1_000,
    );

    let cal = Calibration {
        ..Default::default()
    };

    let mpu = &mut Mpu9250::new(i2c, hal::delay::Delay::new(cp.SYST, clocks), cal).unwrap();
    //let mut ahrs = Marg::new(0.3, 0.01);

    let mut pid = Pid::new(1.0f32, 2.0f32, 3.0f32, 10.0f32, 10.0f32, 10.0f32);

    let mut cnt = 0;
    loop {
        //block!(timer.wait()).unwrap();
        cnt += 1;
        //let start = mono.now();
        if cnt % 200 == 0 {
            led.set_high();
            let _ = writeln!(
                output,
                "angle: {} {} {}",
                mpu.get_accel().unwrap().x,
                mpu.get_accel().unwrap().y,
                mpu.get_accel().unwrap().z
            );
        } else if cnt % 100 == 0 {
            led.set_low();
        }
        //let elapsed = start.elapsed();
        //let _ = writeln!(output, "elapsed: {}", elapsed);
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
