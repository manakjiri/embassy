//! This example runs on a STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LORA P2P send functionality.
#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait, async_fn_in_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_lora::iv::{InterruptHandler, Stm32wlInterfaceVariant};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::spi::Spi;
use embassy_time::{Delay, Timer, Duration, Instant};
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQUENCY_IN_HZ: u32 = 869_525_000; // warning: set this appropriately for the region

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSE;
    let p = embassy_stm32::init(config);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
    let mut debug_indicator = Output::new(p.PB15, Level::Low, Speed::Low);

    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let _ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::High, Speed::High);
    let _ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    let iv = Stm32wlInterfaceVariant::new(Irqs, None, Some(ctrl2)).unwrap();

    let mut lora = {
        match LoRa::new(SX1261_2::new(BoardType::Stm32wlSx1262, spi, iv), false, Delay).await {
            Ok(l) => l,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let mut receiving_buffer = [00u8; 100];

    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_5,
            Bandwidth::_250KHz,
            CodingRate::_4_6,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let mut tx_pkt_params = {
        match lora.create_tx_packet_params(4, false, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };
    match lora.prepare_for_tx(&mdltn_params, 20, false).await {
        Ok(()) => {}
        Err(err) => {
            info!("Radio error = {}", err);
            return;
        }
    };
    
    let mut buffer = [0u8; 100];
    for i in 0..100 {
        buffer[i] = i as u8;
    }
    
    loop {
        debug_indicator.set_high();

        let time_start = Instant::now().as_millis();
        match lora.tx(&mdltn_params, &mut tx_pkt_params, &buffer, 10_000).await {
            Ok(()) => {
                info!("TX done {} ms", Instant::now().as_millis() - time_start);
            }
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        };

        debug_indicator.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }

    
}
