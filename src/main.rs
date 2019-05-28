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
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate embedded_hal;
#[macro_use(block)]
extern crate nb;
extern crate panic_rtt;
extern crate stm32f1xx_hal as hal;

use crate::embedded_hal::spi::MODE_3;
use crate::hal::{i2c::BlockingI2c, prelude::*, qei::Qei, spi::Spi, time::MonoTimer, timer::Timer};
use crate::rt::{entry, ExceptionFrame};

use core::fmt::Write;
use libm::F32Ext;
use madgwick::{F32x3, Marg};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use pid::Pid;
use pscontroller_rs::{dualshock::ControlDS, dualshock::DualShock2, Device, PlayStationPort};
use ssd1306::prelude::*;
use ssd1306::Builder;

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

    // Set up clock tree.
    // * 8MHz external crystal.
    // * System clock at 48MHz (maximum value is 72MHz).
    // * APB1 peripheral clock at 24MHz (maximum value is 36MHz).
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    //=========================================================
    // I2C Bus
    //=========================================================

    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    let i2c = BlockingI2c::i2c2(
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

    let i2c_bus = shared_bus::CortexMBusManager::new(i2c);

    //=========================================================
    // SSD1306 OLED Display
    //=========================================================

    let mut disp: TerminalMode<_> = Builder::new().connect_i2c(i2c_bus.acquire()).into();
    disp.init().unwrap();
    let _ = disp.clear();
    let _ = disp.write_str("Initializing... ");

    //=========================================================
    // LED
    //=========================================================

    let _ = disp.write_char('l');
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let _ = disp.write_char('L');

    //=========================================================
    // SPI Bus
    //=========================================================

    let _ = disp.write_char('s');

    // SPI2
    let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    let miso = gpiob.pb14;
    let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
    let spi = Spi::spi2(
        dp.SPI2,
        (sck, miso, mosi),
        MODE_3,
        250.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let _ = disp.write_char('S');

    //=========================================================
    // Motors
    //=========================================================

    let _ = disp.write_char('m');
    let (motor1_pwm, motor2_pwm, _, _) = dp.TIM4.pwm(
        (
            gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh),
        ),
        &mut afio.mapr,
        2.khz(),
        clocks,
        &mut rcc.apb1,
    );
    let motor1_dir1 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    let motor1_dir2 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let motor2_dir1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let motor2_dir2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

    let mut left_motor = Motor::new(motor1_dir1, motor1_dir2, motor1_pwm);
    let mut right_motor = Motor::new(motor2_dir1, motor2_dir2, motor2_pwm);
    let _ = disp.write_char('M');

    //=========================================================
    // Wheel Encoders
    //=========================================================

    let _ = disp.write_char('e');

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

    let _ = disp.write_char('E');

    //=========================================================
    // MPU 9250 IMU
    //=========================================================

    let _ = disp.write_char('i');

    let cal = Calibration {
        ..Default::default()
    };

    let mut mpu = Mpu9250::new(
        i2c_bus.acquire(),
        hal::delay::Delay::new(cp.SYST, clocks),
        cal,
    )
    .unwrap();
    // DISABLED!! mpu.init().unwrap();
    let mut ahrs = Marg::new(0.3, 0.01);

    let _ = disp.write_char('I');

    //=========================================================
    // PS/2 Joystick
    //=========================================================

    let _ = disp.write_char('j');

    let nss = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    //let nss = gpioa.pa15.into_push_pull_output(&mut gpioa.crh);
    let mut psp = PlayStationPort::new(spi, Some(nss));
    let mut control_ds = ControlDS::new(false, 0);
    let mut big: u8 = 0;
    let mut small: bool = false;
    control_ds.little = small;
    control_ds.big = big;
    psp.enable_pressure().unwrap();

    let _ = disp.write_char('J');

    //=========================================================
    // PID controllers.
    //=========================================================

    let _ = disp.write_char('p');

    // Distance PID takes wheel encoder odometry as input and returns the
    // desired tilt angle in degrees.
    let mut distance_pid = Pid::new(1.0f32, 0.0f32, 1000.0f32, 2.0f32, 2.0f32, 2.0f32);
    distance_pid.update_setpoint(0.0f32);
    let distance_input_multiplier = -0.0001f32;

    // Tilt PID takes tilt angle in degrees as input and returns the desired
    // wheel power on an arbitrary scale of -100 to +100.
    let mut tilt_pid = Pid::new(5.0f32, 0.25f32, 50.0f32, 100.0f32, 100.0f32, 100.0f32);
    tilt_pid.update_setpoint(0.0f32);
    let tilt_input_multiplier = 180.0f32 / core::f32::consts::PI;
    let tilt_output_multiplier = left_motor.get_max_duty() as f32 / 100.0f32;

    let _ = disp.write_char('P');

    //=========================================================
    // Timers for pacing and benchmarking.
    //=========================================================

    let _ = disp.write_char('t');

    // Use TIM1 as our periodic timer.
    let mut timer = Timer::tim1(dp.TIM1, loop_frequency.hz(), clocks, &mut rcc.apb2);

    // Use DWT for benchmarking.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    let mono = MonoTimer::new(cp.DWT, clocks);

    let _ = disp.write_char('T');

    //=========================================================
    // Main loop.
    //=========================================================

    let _ = disp.write_str("Main loop...    ");
    loop {
        //let start = mono.now();

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

        if pitch.abs() > 1.0f32 {
            // If tilt is more than 1 radian (about 50 degrees), stop the motors.
            left_motor.duty(0);
            right_motor.duty(0);
            distance_pid.reset_integral_term();
            tilt_pid.reset_integral_term();
        } else {
            // Set the desired tilt angle, based on the actual vs desired distance travelled.
            left_wheel.update(left_encoder.count());
            right_wheel.update(right_encoder.count());
            let distance_travelled = left_wheel.odometer() + right_wheel.odometer();
            let desired_tilt = distance_pid
                .next_control_output(distance_travelled as f32 * distance_input_multiplier);

            // Set the motor power, based on the actual vs desired tilt.
            tilt_pid.update_setpoint(desired_tilt.output);
            let tilt_output = tilt_pid.next_control_output(pitch * tilt_input_multiplier);
            let speed = (tilt_output.output * tilt_output_multiplier).abs() as u16;
            if tilt_output.output > 0.0f32 {
                left_motor.forward().duty(speed);
                right_motor.forward().duty(speed);
            } else {
                left_motor.reverse().duty(speed);
                right_motor.reverse().duty(speed);
            }
        }

        // let _ = writeln!(
        //     output,
        //     "in/p/i/d/out: {} {} {} {} {}",
        //     distance_travelled as f32 * distance_input_multiplier,
        //     desired_tilt.p,
        //     desired_tilt.i,
        //     desired_tilt.d,
        //     desired_tilt.output,
        // );

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
