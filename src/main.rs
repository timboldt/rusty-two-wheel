#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate jlink_rtt;
extern crate panic_rtt;
extern crate stm32f1xx_hal as hal;
#[macro_use(block)]
extern crate nb;

use crate::hal::delay::Delay;
use crate::hal::i2c::BlockingI2c;
use crate::hal::prelude::*;
use crate::rt::entry;
use crate::rt::ExceptionFrame;

use core::fmt::Write;

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
    let mut timer = hal::timer::Timer::syst(cp.SYST, 1000.hz(), clocks);

    //=========================================================
    // I2C Bus (for MPU 9250)
    //=========================================================

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

    let mut mono = ::hal::time::MonoTimer::new(cp.DWT, clocks);
    let start = mono.now();

    let buffer: &mut [u8] = &mut [0u8; 1];
    const MPU9250_DEVICE_ADDRESS: u8 = 0x68;
    const MPU9250_RA_WHO_AM_I: u8 = 0x75;
    i2c.write_read(MPU9250_DEVICE_ADDRESS, &[MPU9250_RA_WHO_AM_I], buffer)
        .unwrap();
    const MPU9250_WHOAMI_ID: u8 = 0x71;
    if buffer[0] != MPU9250_WHOAMI_ID {
        panic!("Invalid WHO_AM_I_ID.");
    }

    let elapsed = start.elapsed();
    let _ = writeln!(output, "elapsed: {}", elapsed);

    let mut cnt = 0;
    loop {
        block!(timer.wait()).unwrap();
        cnt += 1;
        let start = mono.now();
        if cnt % 200 == 0 {
            led.set_high();
        } else if cnt % 100 == 0 {
            led.set_low();
        }
        let elapsed = start.elapsed();
        let _ = writeln!(output, "elapsed: {}", elapsed);
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
