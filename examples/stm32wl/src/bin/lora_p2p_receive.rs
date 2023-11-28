//! This example runs on the STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LORA P2P receive functionality in conjunction with the lora_p2p_send example.
#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait, async_fn_in_trait)]
#![allow(stable_features, unknown_lints, async_fn_in_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_lora::iv::{InterruptHandler, Stm32wlInterfaceVariant};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::spi::{Spi, Config};
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Timer};
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
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(32_000_000),
            mode: HseMode::Bypass,
            prescaler: HsePrescaler::DIV1,
        });
        config.rcc.mux = ClockSrc::PLL1_R;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL6,
            divp: None,
            divq: Some(PllQDiv::DIV2), // PLL1_Q clock (32 / 2 * 6 / 2), used for RNG
            divr: Some(PllRDiv::DIV2), // sysclk 48Mhz clock (32 / 2 * 6 / 2)
        });
    }
    let p = embassy_stm32::init(config);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);

    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let ctrl2 = Output::new(p.PA9.degrade(), Level::High, Speed::High);
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

    //let mut spi_config = Config::default();
    //spi_config.frequency = Hertz(1_000_000);
    //let mut _spi = Spi::new(p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA2_CH3, p.DMA2_CH2, spi_config);
    //let spi_data = [1u8, 2u8, 3u8];
    //_spi.write(&spi_data).await.ok();

    let mut debug_indicator = Output::new(p.PC13, Level::Low, Speed::Low);
    //let mut start_indicator = Output::new(p.PB9, Level::Low, Speed::Low);

    debug_indicator.set_high();
    Timer::after(Duration::from_secs(5)).await;
    debug_indicator.set_low();
    
    //start_indicator.set_high();
    //Timer::after_secs(5).await;
    //start_indicator.set_low();

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

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let mut buffer = [0u8; 100];
    for i in 0..100 {
        buffer[i] = i as u8;
    }

    loop {

        match lora
            .prepare_for_rx(&mdltn_params, &rx_pkt_params, None, None, false)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        };

        receiving_buffer = [00u8; 100];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, rx_pkt_status)) => {
                if received_len == 100 && receiving_buffer.iter().eq(buffer.iter())
                {
                    debug_indicator.set_high();
                    info!("rx snr {} rssi {}", rx_pkt_status.snr, rx_pkt_status.rssi);

                    Timer::after(Duration::from_millis(100)).await;
                    //Timer::after_secs(5).await;
                    debug_indicator.set_low();
                } else {
                    info!("rx unknown packet");
                }
            }
            Err(err) => info!("rx unsuccessful = {}", err),
        }
    }
}
