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
extern crate libm;
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
use libm::F32Ext;
use madgwick::{F32x3, Marg};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use pid::Pid;

mod motor;
mod wheel;

use motor::Motor;
use wheel::Wheel;

#[entry]
fn main() -> ! {
    // Various constants.
    let loop_frequency = 200;
    let loop_millis = 1000 / loop_frequency;

    #[allow(unused_mut)]
    #[allow(unused_variables)]
    let mut output = jlink_rtt::NonBlockingOutput::new();

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
    let mut left_wheel = Wheel::new(left_encoder.count(), loop_millis);

    // TIM3
    let c1 = gpioa.pa6;
    let c2 = gpioa.pa7;
    let right_encoder = Qei::tim3(dp.TIM3, (c1, c2), &mut afio.mapr, &mut rcc.apb1);
    let mut right_wheel = Wheel::new(right_encoder.count(), loop_millis);

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

    let mut mpu = Mpu9250::new(i2c, hal::delay::Delay::new(cp.SYST, clocks), cal).unwrap();
    mpu.init().unwrap();
    let mut ahrs = Marg::new(0.3, 0.01);

    //=========================================================
    // PID controllers.
    //=========================================================

    // Output is an arbitrary scale of -100 to +100.
    let mut tilt_pid = Pid::new(5.0f32, 0.25f32, 50.0f32, 100.0f32, 100.0f32, 100.0f32);
    tilt_pid.update_setpoint(0.0f32);
    let tilt_input_multiplier = 180.0f32 / core::f32::consts::PI;
    let tilt_output_multiplier = left_motor.get_max_duty() as f32 / 100.0f32;

    //=========================================================
    // Timers for pacing and benchmarking.
    //=========================================================

    // Use TIM1 as our periodic timer.
    let mut timer = Timer::tim1(dp.TIM1, loop_frequency.hz(), clocks, &mut rcc.apb2);

    // Use DWT for benchmarking.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    let mono = MonoTimer::new(cp.DWT, clocks);

    //=========================================================
    // Main loop.
    //=========================================================

    writeln!(output, "Entering main loop...");
    loop {
        // let start = mono.now();

        let (va, vg) = mpu.get_accel_gyro().unwrap();
        let vm = mpu.get_mag().unwrap();
        let deg_to_rad = core::f32::consts::PI / 180.0f32;
        let q = ahrs.update(
            F32x3 {
                x: vm.x,
                y: vm.y,
                z: vm.z,
            },
            F32x3 {
                x: vg.x * deg_to_rad,
                y: vg.y * deg_to_rad,
                z: vg.z * deg_to_rad,
            },
            F32x3 {
                x: va.x,
                y: va.y,
                z: va.z,
            },
        );
        // let roll = f32::atan2(
        //     2.0f32 * (q.0 * q.1 + q.2 * q.3),
        //     1.0f32 - 2.0f32 * (q.1 * q.1 + q.2 * q.2),
        // );
        let pitch = f32::asin(2.0f32 * (q.0 * q.2 - q.1 * q.3));
        // let yaw = f32::atan2(
        //     2.0f32 * (q.0 * q.3 + q.1 * q.2),
        //     1.0f32 - 2.0f32 * (q.2 * q.2 + q.3 * q.3),
        // );
        //let _ = writeln!(output, "r/p/y: {} {} {} {}", roll, pitch, yaw, elapsed);

        let tilt_output = tilt_pid.next_control_output(pitch * tilt_input_multiplier);
        let speed = (tilt_output.output * tilt_output_multiplier).abs() as u16;
        if tilt_output.output > 0.0f32 {
            left_motor.forward().duty(speed);
            right_motor.forward().duty(speed);
        } else {
            left_motor.reverse().duty(speed);
            right_motor.reverse().duty(speed);
        }

        // let elapsed = start.elapsed();
        // let _ = writeln!(
        //     output,
        //     "ptch/p/i/d/out/elap: {} {} {} {} {} {}",
        //     pitch * tilt_input_multiplier,
        //     tilt_output.p,
        //     tilt_output.i,
        //     tilt_output.d,
        //     tilt_output.output * tilt_output_multiplier,
        //     elapsed
        // );

        // left_wheel.update(left_encoder.count());
        // right_wheel.update(right_encoder.count());
        // let _ = writeln!(
        //     output,
        //     "lo/ro/lv/rv: {} {} {} {}",
        //     left_wheel.odometer(),
        //     right_wheel.odometer(),
        //     left_wheel.velocity(),
        //     right_wheel.velocity()
        // );

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
