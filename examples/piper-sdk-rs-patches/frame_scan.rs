//! frame_scan — passively reads ~200 CAN frames from the Piper and prints
//! the unique CAN IDs with hit counts and a sample data row each. Helps
//! reverse-engineer firmware protocol changes (e.g. S-V1.8-2 vs S-V1.8-3).
//!
//! Read-only. No motors enabled. No commands sent. Safe to run any time
//! the arm is powered on and CAN is connected.
//!
//! Run:
//!   sudo ./target/debug/examples/frame_scan
//!   sudo ./target/debug/examples/frame_scan --frames 500

use rusb::{Device, DeviceHandle, GlobalContext, TransferType};
use std::collections::BTreeMap;
use std::time::Duration;

const GS_USB_ID_VENDOR: u16 = 0x1D50;
const GS_USB_ID_PRODUCT: u16 = 0x606F;
const GS_USB_CANDLELIGHT_VENDOR_ID: u16 = 0x1209;
const GS_USB_CANDLELIGHT_PRODUCT_ID: u16 = 0x2323;

const GS_USB_REQ_OUT: u8 = 0x41;
const GS_USB_REQ_IN: u8 = 0xC1;
const GS_USB_BREQ_BITTIMING: u8 = 1;
const GS_USB_BREQ_MODE: u8 = 2;
const GS_USB_BREQ_BT_CONST: u8 = 4;

const GS_CAN_MODE_NORMAL: u32 = 0;
const GS_CAN_MODE_HW_TIMESTAMP: u32 = 1 << 4;
const GS_CAN_MODE_RESET: u32 = 0;
const GS_CAN_MODE_START: u32 = 1;

const GS_USB_FRAME_SIZE_HW_TIMESTAMP: usize = 24;

fn is_gs_usb_device(vendor_id: u16, product_id: u16) -> bool {
    matches!(
        (vendor_id, product_id),
        (GS_USB_ID_VENDOR, GS_USB_ID_PRODUCT)
            | (GS_USB_CANDLELIGHT_VENDOR_ID, GS_USB_CANDLELIGHT_PRODUCT_ID)
    )
}

fn find_device() -> Option<(Device<GlobalContext>, DeviceHandle<GlobalContext>)> {
    let devices = rusb::devices().ok()?;
    for device in devices.iter() {
        let Ok(desc) = device.device_descriptor() else {
            continue;
        };
        if is_gs_usb_device(desc.vendor_id(), desc.product_id())
            && let Ok(handle) = device.open()
        {
            return Some((device, handle));
        }
    }
    None
}

fn find_bulk_endpoints(device: &Device<GlobalContext>) -> Option<(u8, u8)> {
    let cfg = device.active_config_descriptor().ok()?;
    for interface in cfg.interfaces() {
        for desc in interface.descriptors() {
            if desc.interface_number() != 0 {
                continue;
            }
            let mut ep_in = None;
            let mut ep_out = None;
            for ep in desc.endpoint_descriptors() {
                if ep.transfer_type() != TransferType::Bulk {
                    continue;
                }
                match ep.direction() {
                    rusb::Direction::In => ep_in = Some(ep.address()),
                    rusb::Direction::Out => ep_out = Some(ep.address()),
                }
            }
            if let (Some(i), Some(o)) = (ep_in, ep_out) {
                return Some((i, o));
            }
        }
    }
    None
}

fn set_bitrate_1m(handle: &DeviceHandle<GlobalContext>) -> Result<(), rusb::Error> {
    if handle.kernel_driver_active(0).unwrap_or(false) {
        handle.detach_kernel_driver(0)?;
    }
    handle.claim_interface(0)?;
    let mut buf = vec![0u8; 40];
    let len = handle.read_control(
        GS_USB_REQ_IN, GS_USB_BREQ_BT_CONST, 0, 0, &mut buf,
        Duration::from_millis(1000),
    )?;
    if len < 40 {
        return Err(rusb::Error::Other);
    }
    let fclk = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let (prop_seg, phase_seg1, phase_seg2, sjw, brp) = match fclk {
        48_000_000 => (1u32, 12u32, 2u32, 1u32, 3u32),
        80_000_000 => (1u32, 12u32, 2u32, 1u32, 5u32),
        _ => return Err(rusb::Error::Other),
    };
    let mut bt = [0u8; 20];
    bt[0..4].copy_from_slice(&prop_seg.to_le_bytes());
    bt[4..8].copy_from_slice(&phase_seg1.to_le_bytes());
    bt[8..12].copy_from_slice(&phase_seg2.to_le_bytes());
    bt[12..16].copy_from_slice(&sjw.to_le_bytes());
    bt[16..20].copy_from_slice(&brp.to_le_bytes());
    handle.write_control(
        GS_USB_REQ_OUT, GS_USB_BREQ_BITTIMING, 0, 0, &bt,
        Duration::from_millis(1000),
    )?;
    Ok(())
}

fn set_mode(handle: &DeviceHandle<GlobalContext>, mode: u32, flags: u32) -> Result<(), rusb::Error> {
    let mut payload = [0u8; 8];
    payload[0..4].copy_from_slice(&mode.to_le_bytes());
    payload[4..8].copy_from_slice(&flags.to_le_bytes());
    handle.write_control(
        GS_USB_REQ_OUT, GS_USB_BREQ_MODE, 0, 0, &payload,
        Duration::from_millis(1000),
    )?;
    Ok(())
}

#[derive(Debug, Clone)]
struct IdStat {
    count: u32,
    last_data: [u8; 8],
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let target_frames: usize = std::env::args()
        .skip(1)
        .find_map(|a| a.strip_prefix("--frames=").map(str::to_owned))
        .or_else(|| {
            let args: Vec<String> = std::env::args().collect();
            args.iter().position(|a| a == "--frames").and_then(|i| args.get(i + 1).cloned())
        })
        .and_then(|v| v.parse().ok())
        .unwrap_or(200);

    println!("frame_scan — reading {} CAN frames passively", target_frames);

    let (device, handle) = find_device().ok_or("GS-USB device not found")?;
    let (ep_in, _) = find_bulk_endpoints(&device).ok_or("Bulk endpoints not found")?;
    println!("opened candleLight, ep_in=0x{:02X}", ep_in);

    handle.reset()?;
    set_bitrate_1m(&handle)?;
    set_mode(&handle, GS_CAN_MODE_START, GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP)?;
    std::thread::sleep(Duration::from_millis(200));

    let mut stats: BTreeMap<u32, IdStat> = BTreeMap::new();
    let mut buf = vec![0u8; GS_USB_FRAME_SIZE_HW_TIMESTAMP];
    let mut total = 0usize;
    let mut errs = 0usize;

    while total < target_frames {
        match handle.read_bulk(ep_in, &mut buf, Duration::from_secs(2)) {
            Ok(len) if len >= GS_USB_FRAME_SIZE_HW_TIMESTAMP => {
                let can_id = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]) & 0x1FFFFFFF;
                let mut data = [0u8; 8];
                data.copy_from_slice(&buf[12..20]);
                stats
                    .entry(can_id)
                    .and_modify(|s| {
                        s.count += 1;
                        s.last_data = data;
                    })
                    .or_insert(IdStat { count: 1, last_data: data });
                total += 1;
            }
            Ok(_) => errs += 1,
            Err(rusb::Error::Timeout) => {
                eprintln!("read timeout — no more frames (total={})", total);
                break;
            }
            Err(e) => {
                eprintln!("read err: {e}");
                errs += 1;
                if errs > 10 {
                    break;
                }
            }
        }
    }

    let _ = set_mode(&handle, GS_CAN_MODE_RESET, 0);

    println!("\n=== summary ===");
    println!("total frames: {}", total);
    println!("unique IDs:   {}", stats.len());
    println!("read errors:  {}", errs);
    println!();
    println!("{:>6}  {:>6}  {}", "ID(hex)", "count", "last data");
    println!("{}", "-".repeat(60));
    for (id, stat) in &stats {
        println!(
            "0x{:03X}  {:>6}  {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
            id, stat.count,
            stat.last_data[0], stat.last_data[1], stat.last_data[2], stat.last_data[3],
            stat.last_data[4], stat.last_data[5], stat.last_data[6], stat.last_data[7],
        );
    }

    Ok(())
}
