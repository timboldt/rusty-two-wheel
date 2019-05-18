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
extern crate jlink_rtt;
extern crate madgwick;
extern crate mpu9250_i2c;
#[macro_use(block)]
extern crate nb;
extern crate panic_rtt;
extern crate pid;
extern crate stm32f1xx_hal as hal;

//use crate::hal::delay::Delay;
use crate::hal::{i2c::BlockingI2c, prelude::*, qei::Qei, time::MonoTimer, timer::Timer};
use crate::rt::{entry, ExceptionFrame};

use core::fmt::Write;
use madgwick::{F32x3, Marg};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use pid::Pid;

mod motor;
use motor::Motor;

#[entry]
fn main() -> ! {
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    let mut output = jlink_rtt::Output::new();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    //=========================================================
    // Clocks
    //=========================================================

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

    //=========================================================
    // LED
    //=========================================================

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    //=========================================================
    // Motors
    //=========================================================

    let (motor1_pwm, motor2_pwm, _, _) = dp.TIM4.pwm(
        (
            gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh),
        ),
        &mut afio.mapr,
        1.khz(),
        clocks,
        &mut rcc.apb1,
    );
    let motor1_dir1 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    let motor1_dir2 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let motor2_dir1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let motor2_dir2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

    let mut left_motor = Motor::new(motor1_dir1, motor1_dir2, motor1_pwm);
    let mut right_motor = Motor::new(motor2_dir1, motor2_dir2, motor2_pwm);

    //=========================================================
    // Wheel Encoders
    //=========================================================

    // TIM2
    let c1 = gpioa.pa0;
    let c2 = gpioa.pa1;
    let left_encoder = Qei::tim2(dp.TIM2, (c1, c2), &mut afio.mapr, &mut rcc.apb1);

    // TIM3
    let c1 = gpioa.pa6;
    let c2 = gpioa.pa7;
    let right_encoder = Qei::tim3(dp.TIM3, (c1, c2), &mut afio.mapr, &mut rcc.apb1);

    //=========================================================
    // MPU 9250 IMU (using I2C, for now)
    //=========================================================

    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    let i2c = BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        hal::i2c::Mode::Fast {
            frequency: 100_000,
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

    //let mut pid = Pid::new(1.0f32, 2.0f32, 3.0f32, 10.0f32, 10.0f32, 10.0f32);

    //=========================================================
    // Timers for pacing and benchmarking.
    //=========================================================

    // Use TIM1 as our periodic timer.
    let mut timer = Timer::tim1(dp.TIM1, 200.hz(), clocks, &mut rcc.apb2);

    // Use DWT for benchmarking.
    //let mono = MonoTimer::new(cp.DWT, clocks);
    //let start = mono.now();
    // ... do something ...
    //let elapsed = start.elapsed();
    //let _ = writeln!(output, "elapsed: {}", elapsed);

    //=========================================================
    // Main loop.
    //=========================================================

    loop {
        let (_va, _vg) = mpu.get_accel_gyro().unwrap();
        // let _ = writeln!(
        //     output,
        //     "a/g: {} {} {} {} {} {}",
        //     va.x, va.y, va.z, vg.x, vg.y, vg.z,
        // );

        let left_odometer = left_encoder.count();
        let right_odometer = right_encoder.count();
        let _ = writeln!(output, "le/re: {} {}", left_odometer, right_odometer);

        let speed = left_motor.get_max_duty() / 2;
        left_motor.forward().duty(speed);
        right_motor.duty(speed).reverse();

        // We turn the LED on during the wait, which means the brightness is
        // proportional to the idle time.
        #[allow(deprecated)]
        led.set_low(); // Inverted logic: ON.

        // Wait for next period to start.
        block!(timer.wait()).unwrap();

        #[allow(deprecated)]
        led.set_high(); // Inverted logic: OFF.
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
