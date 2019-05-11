//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manaual for an explanation. This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

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
    let mut timer = Timer::syst(cp.SYST, 1000.hz(), clocks);

    let mut cnt = 0;
    loop {
        block!(timer.wait()).unwrap();
        cnt += 1;
        if cnt % 200 == 0 {
            led.set_high();
        } else if cnt % 100 == 0 {
            led.set_low();
        }
    }
}
