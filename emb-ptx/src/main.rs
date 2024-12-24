#![no_std]
#![no_main]

use core::{ptr::null_mut, sync::atomic::{AtomicBool, AtomicPtr, Ordering}};

use cortex_m::peripheral::NVIC;
use embassy_nrf::{config::{Config, HfclkSource}, interrupt, pac::Interrupt};
use embassy_executor::Spawner;
use embassy_nrf::{
    gpio::{Level, Output, OutputDrive},
    pac::RADIO,
};
use embassy_time::{Duration, Ticker, Timer};
use esb::{
    bbq2::queue::BBQueue, irq::StatePTX, peripherals::{PtrTimer, Timer0}, Addresses, ConfigBuilder, Error, EsbBuffer, EsbHeader, EsbIrq, IrqTimer
};
use mutex::{raw_impls::cs::CriticalSectionRawMutex, BlockingMutex};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const MAX_PAYLOAD_SIZE: u8 = 64;
const MSG: &'static str = "Hello from PTX";

type IrqStorage = BlockingMutex<CriticalSectionRawMutex, EsbIrq<1024, 1024, Timer0, StatePTX>>;
static ESB_IRQ: StaticCell<IrqStorage> = StaticCell::new();
static IRQ_PTR: AtomicPtr<IrqStorage> = AtomicPtr::new(null_mut());

type TimerStorage = BlockingMutex<CriticalSectionRawMutex, IrqTimer<Timer0>>;
static TIM_IRQ: StaticCell<TimerStorage> = StaticCell::new();
static TIM_PTR: AtomicPtr<TimerStorage> = AtomicPtr::new(null_mut());

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut c = Config::default();
    c.hfclk_source = HfclkSource::ExternalXtal;
    let _p = embassy_nrf::init(c);
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // let mut led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    static BUFFER: EsbBuffer<1024, 1024> = EsbBuffer {
        app_to_radio_buf: BBQueue::new(),
        radio_to_app_buf: BBQueue::new(),
        timer_flag: AtomicBool::new(false),
    };
    let addresses = Addresses::default();
    let config = ConfigBuilder::default()
        .maximum_transmit_attempts(0)
        .max_payload_size(MAX_PAYLOAD_SIZE)
        .check()
        .unwrap();
    let (mut esb_app, esb_irq, esb_timer) = BUFFER
        .try_split(unsafe { Timer0::take() }, RADIO, addresses, config)
        .unwrap();

    let mut pid = 0;
    let mut ct_tx: u32 = 0;
    let mut ct_rx: u32 = 0;
    let mut ct_err: u32 = 0;

    let esb_irq = esb_irq.into_ptx();
    {
        let irq_ref = ESB_IRQ.init(BlockingMutex::new(esb_irq));
        IRQ_PTR.store(irq_ref, Ordering::Release);
    }
    {
        let tim_ref = TIM_IRQ.init(BlockingMutex::new(esb_timer));
        TIM_PTR.store(tim_ref, Ordering::Release);
    }
    unsafe {
        cp.NVIC.set_priority(Interrupt::TIMER0, 0);
        cp.NVIC.set_priority(Interrupt::RADIO, 0);
        NVIC::unmask(Interrupt::TIMER0);
        NVIC::unmask(Interrupt::RADIO);
    }

    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        let esb_header = EsbHeader::build()
            .max_payload(MAX_PAYLOAD_SIZE)
            .pid(pid)
            .pipe(0)
            .no_ack(false)
            .check()
            .unwrap();
        if pid == 3 {
            pid = 0;
        } else {
            pid += 1;
        }

        // Did we receive any packet ?
        if let Some(response) = esb_app.read_packet() {
            ct_rx += 1;
            let text = core::str::from_utf8(&response[..]).unwrap();
            let rssi = response.get_header().rssi();
            defmt::info!("Payload: '{=str}', RSSI: {=u8}", text, rssi);
            // write!(serial, "\rPayload: ").unwrap();

            // let text = ;
            // serial.write_str(text).unwrap();
            // //serial.write(&response[..]).unwrap();
            // let rssi = response.get_header().rssi();
            // write!(serial, " | rssi: {}", rssi).unwrap();
            response.release();
        } else if ct_tx != 0 {
            defmt::info!("No packet?");
            ct_err += 1;
        }

        defmt::info!("Sending Hello, tx: {=u32}, rx: {=u32}, err: {=u32}", ct_tx, ct_rx, ct_err);

        ct_tx += 1;
        let mut packet = esb_app.grant_packet(esb_header).unwrap();
        let length = MSG.as_bytes().len();
        packet[..length].copy_from_slice(MSG.as_bytes());
        packet.commit(length);
        esb_app.start_tx();

    }
}

#[interrupt]
fn RADIO() {
    let ptr = IRQ_PTR.load(Ordering::Relaxed);
    let r = unsafe { ptr.as_ref() }.unwrap();
    let s = r.with_lock(|state| {
        match state.radio_interrupt() {
            Ok(s) => Some(s),
            Err(Error::MaximumAttempts) => None,
            Err(_e) => panic!(),
        }
    });
    if let Some(_s) = s {
        // match s {
        //     StatePTX::IdleTx => defmt::info!("IdleTx"),
        //     StatePTX::TransmitterTx => defmt::info!("TransmitterTx"),
        //     StatePTX::TransmitterTxNoAck => defmt::info!("TransmitterTxNoAck"),
        //     StatePTX::TransmitterWaitAck => defmt::info!("TransmitterWaitAck"),
        //     StatePTX::TransmitterWaitRetransmit => defmt::info!("TransmitterWaitRetransmit"),
        // }
    } else {
        defmt::info!("MAX ATTEMPTS");
    }
}

#[interrupt]
fn TIMER0() {
    let ptr = TIM_PTR.load(Ordering::Relaxed);
    let r = unsafe { ptr.as_ref() }.unwrap();
    r.with_lock(|state| {
        state.timer_interrupt();
    });
}

// #[idle(resources = [serial, esb_app])]
// fn idle(ctx: idle::Context) -> ! {

// }

// #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
// fn radio(ctx: radio::Context) {
//     match esb_irq.radio_interrupt() {
//         Err(Error::MaximumAttempts) => {
//             ATTEMPTS_FLAG.store(true, Ordering::Release);
//         }
//         Err(e) => panic!("Found error {:?}", e),
//         Ok(_) => {} //rinfo!("{:?}", state),
//     }
// }

// #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
// fn timer0(ctx: timer0::Context) {
//     esb_timer.timer_interrupt();
// }
