//! 退出拖动示教模式
//!
//! Sends a raw EmergencyStopCommand::resume() frame (CAN ID 0x150, data=[0x02, 0, …, 0])
//! over GS-USB. This clears `teach_status` so the arm honors subsequent mode commands.
//!
//! Run before `position_control_demo` if the arm is stuck in teach mode (control_mode=2).
//!
//!   sudo cargo run -p piper-sdk --example exit_teach_mode

use rusb::{Device, DeviceHandle, GlobalContext, TransferType};
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
const GS_CAN_MODE_RESET: u32 = 0;
const GS_CAN_MODE_START: u32 = 1;

const PIPER_EMERGENCY_STOP_ID: u32 = 0x150;
const EMERGENCY_ACTION_RESUME: u8 = 0x02;
const TEACH_COMMAND_END_RECORD: u8 = 0x02;

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

/// Discover bulk IN/OUT endpoints on interface 0, alt setting 0.
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

    // Read device capability to pick timing
    let mut buf = vec![0u8; 40];
    let len = handle.read_control(
        GS_USB_REQ_IN,
        GS_USB_BREQ_BT_CONST,
        0,
        0,
        &mut buf,
        Duration::from_millis(1000),
    )?;
    if len < 40 {
        return Err(rusb::Error::Other);
    }
    let fclk = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);

    // 1 Mbps timing for 48 MHz or 80 MHz clock (from gs_usb_direct_test)
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
        GS_USB_REQ_OUT,
        GS_USB_BREQ_BITTIMING,
        0,
        0,
        &bt,
        Duration::from_millis(1000),
    )?;
    Ok(())
}

fn set_mode(handle: &DeviceHandle<GlobalContext>, mode: u32, flags: u32) -> Result<(), rusb::Error> {
    let mut payload = [0u8; 8];
    payload[0..4].copy_from_slice(&mode.to_le_bytes());
    payload[4..8].copy_from_slice(&flags.to_le_bytes());
    handle.write_control(
        GS_USB_REQ_OUT,
        GS_USB_BREQ_MODE,
        0,
        0,
        &payload,
        Duration::from_millis(1000),
    )?;
    Ok(())
}

/// Build a standard CAN frame in GS-USB wire format (20 bytes, no hw_timestamp on TX).
fn build_tx_frame(echo_id: u32, can_id: u32, data: [u8; 8]) -> [u8; 20] {
    let mut buf = [0u8; 20];
    buf[0..4].copy_from_slice(&echo_id.to_le_bytes());
    buf[4..8].copy_from_slice(&can_id.to_le_bytes());
    buf[8] = 8; // can_dlc
    buf[9] = 0; // channel
    buf[10] = 0; // flags
    buf[11] = 0; // reserved
    buf[12..20].copy_from_slice(&data);
    buf
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("{}", "=".repeat(60));
    println!("Exit Teach Mode — sends EmergencyStopCommand::resume()");
    println!("{}", "=".repeat(60));

    println!("\n[1] Finding GS-USB device...");
    let (device, handle) = find_device().ok_or("GS-USB device not found")?;
    let (ep_in, ep_out) = find_bulk_endpoints(&device).ok_or("Bulk endpoints not found")?;
    println!("    ✓ Found (ep_in=0x{:02X}, ep_out=0x{:02X})", ep_in, ep_out);

    println!("\n[2] Resetting + claiming interface, setting 1 Mbps...");
    handle.reset()?;
    set_bitrate_1m(&handle)?;
    println!("    ✓ Bitrate set");

    println!("\n[3] Starting CAN device (normal mode)...");
    set_mode(&handle, GS_CAN_MODE_START, GS_CAN_MODE_NORMAL)?;
    std::thread::sleep(Duration::from_millis(200));
    println!("    ✓ Started");

    // EmergencyStopCommand byte layout:
    //   byte 0: emergency_stop (0x02 = Resume — clears any latched estop)
    //   byte 1: trajectory_command (0x00 = Closed)
    //   byte 2: teach_command     (0x02 = EndRecord — exit drag-teach)
    //   bytes 3-7: 0
    let data = [
        EMERGENCY_ACTION_RESUME,
        0x00,
        TEACH_COMMAND_END_RECORD,
        0, 0, 0, 0, 0,
    ];
    println!(
        "\n[4] Sending exit-teach frame (CAN 0x150 data={:02x?})...",
        data
    );
    let mut sent = 0;
    for i in 0..10 {
        let frame = build_tx_frame(i as u32, PIPER_EMERGENCY_STOP_ID, data);
        match handle.write_bulk(ep_out, &frame, Duration::from_millis(500)) {
            Ok(_) => {
                sent += 1;
                println!("    sent #{}", i + 1);
            },
            Err(e) => {
                println!("    send #{} failed: {} (continuing)", i + 1, e);
            },
        }
        std::thread::sleep(Duration::from_millis(80));
    }
    println!("    ✓ {} frames sent", sent);

    std::thread::sleep(Duration::from_millis(300));

    println!("\n[5] Stopping CAN device...");
    let _ = set_mode(&handle, GS_CAN_MODE_RESET, 0);
    println!("    ✓ Stopped");

    println!("\n{}", "=".repeat(60));
    println!("Done. Now re-run position_control_demo to check control_mode.");
    println!("{}", "=".repeat(60));

    Ok(())
}
