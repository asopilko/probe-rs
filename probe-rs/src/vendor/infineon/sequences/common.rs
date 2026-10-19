//! Helpers shared by the PSOC C3 and PSOC Edge debug sequences.

use std::{
    error::Error,
    thread,
    time::{Duration, Instant},
};

use crate::MemoryMappedRegister;
use crate::architecture::arm::{
    ApV2Address, ArmDebugInterface, ArmError, DapAccess, DapError, FullyQualifiedApAddress, Pins,
    RegisterAddress,
    core::armv7m::{Aircr, Dhcsr},
    dp::{Abort, Ctrl, DPIDR, DpAddress, DpRegister},
    memory::ArmMemoryInterface,
    sequences::{ArmDebugSequence, DefaultArmSequence},
    traits::DebugPortWire,
};
use crate::probe::{BitSequence, WireProtocol, swd::Port};

// APv2 MEM-AP register offsets.
pub(super) const AP_CSW: u64 = 0xD00;
pub(super) const AP_TAR: u64 = 0xD04;
pub(super) const AP_DRW: u64 = 0xD0C;

/// Bus-access AP, reachable regardless of the core APs' lock state.
pub(super) const SYS_AP_BASE: u64 = 0xF000_0000;

pub(super) const CM33_AP_BASE: u64 = 0xF000_2000;

/// Time the boot ROM needs to reopen the debug window after a reset.
pub(super) const RESET_FINISH_DELAY_MS: u64 = 400;

pub(super) fn sys_ap(dp: DpAddress) -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v2_with_dp(dp, ApV2Address(Some(SYS_AP_BASE)))
}

pub(super) fn cm33(dp: DpAddress) -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v2_with_dp(dp, ApV2Address(Some(CM33_AP_BASE)))
}

/// Over JTAG the SWJ-DP NACKs DPACC reads until powered up, so post the power-up write.
pub(super) fn jtag_dp_powerup(interface: &mut dyn DapAccess, dp: DpAddress, prime_abort: bool) {
    let is_jtag = interface.active_wire_protocol() == Some(WireProtocol::Jtag);
    if !is_jtag {
        return;
    }

    // Flushes the stale ACK shifted out on the first posted DPACC access.
    if prime_abort {
        let mut abort = Abort(0);
        abort.set_orunerrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);
        abort.set_wderrclr(true);
        let _ = interface.write_raw_dp_register(dp, Abort::ADDRESS, abort.into());
    }

    // CSYSPWRUPREQ | CDBGPWRUPREQ | sticky-error clears; no MASKLANE on a MinDP.
    let _ = interface.write_raw_dp_register(dp, Ctrl::ADDRESS, 0x5000_0032);
}

pub(super) fn jtag_dp_start_with_powerup(
    interface: &mut dyn DapAccess,
    dp: DpAddress,
    prime_abort: bool,
    prime_before_first: bool,
) -> Result<(), ArmError> {
    if prime_before_first {
        jtag_dp_powerup(interface, dp, prime_abort);
    }

    if DefaultArmSequence(())
        .debug_port_start(&mut *interface, dp)
        .is_ok()
    {
        return Ok(());
    }

    jtag_dp_powerup(interface, dp, prime_abort);
    DefaultArmSequence(()).debug_port_start(interface, dp)
}

/// Over SWD the CTRL/STAT sticky bits are read-only, so clear them through `ABORT`.
pub(super) fn swd_dp_clear_sticky(interface: &mut dyn DapAccess, dp: DpAddress) {
    let is_jtag = interface.active_wire_protocol() == Some(WireProtocol::Jtag);
    if is_jtag {
        return;
    }

    let mut abort = Abort(0);
    abort.set_orunerrclr(true);
    abort.set_wderrclr(true);
    abort.set_stkerrclr(true);
    abort.set_stkcmpclr(true);
    let _ = interface.write_raw_dp_register(dp, Abort::ADDRESS, abort.into());
}

pub(super) fn swd_dp_start_with_powerup(
    interface: &mut dyn DapAccess,
    dp: DpAddress,
) -> Result<(), ArmError> {
    swd_dp_clear_sticky(interface, dp);

    if DefaultArmSequence(())
        .debug_port_start(&mut *interface, dp)
        .is_ok()
    {
        return Ok(());
    }

    swd_dp_clear_sticky(interface, dp);
    DefaultArmSequence(()).debug_port_start(interface, dp)
}

pub(super) fn dp_start_with_powerup(
    interface: &mut dyn DapAccess,
    dp: DpAddress,
    prime_abort: bool,
    prime_before_first: bool,
) -> Result<(), ArmError> {
    let is_jtag = interface.active_wire_protocol() == Some(WireProtocol::Jtag);
    if is_jtag {
        jtag_dp_start_with_powerup(interface, dp, prime_abort, prime_before_first)
    } else {
        swd_dp_start_with_powerup(interface, dp)
    }
}

/// Raw TAR/DRW access: `memory_interface()` resets CSW and clears the SYS AP's `DbgSwEnable`.
pub(super) fn write_mem32(
    iface: &mut dyn ArmDebugInterface,
    ap: &FullyQualifiedApAddress,
    addr: u32,
    val: u32,
) -> Result<(), ArmError> {
    iface.write_raw_ap_register(ap, AP_TAR, addr)?;
    iface.write_raw_ap_register(ap, AP_DRW, val)?;
    Ok(())
}

pub(super) fn read_mem32(
    iface: &mut dyn ArmDebugInterface,
    ap: &FullyQualifiedApAddress,
    addr: u32,
) -> Result<u32, ArmError> {
    iface.write_raw_ap_register(ap, AP_TAR, addr)?;
    iface.read_raw_ap_register(ap, AP_DRW)
}

pub(super) fn clear_dp_sticky_errors(
    interface: &mut dyn DapAccess,
    dp: DpAddress,
) -> Result<(), ArmError> {
    let mut abort = Abort(0);
    abort.set_orunerrclr(true);
    abort.set_wderrclr(true);
    abort.set_stkerrclr(true);
    abort.set_stkcmpclr(true);
    interface.write_raw_dp_register(dp, Abort::ADDRESS, abort.0)
}

/// Pulses XRES so the boot ROM re-enables the SWJ pins; `false` if nRESET can't be driven.
pub(super) fn pulse_xres_for(interface: &mut dyn DebugPortWire, hold_ms: u64) -> bool {
    let mut n_reset = Pins(0);
    n_reset.set_nreset(true);

    // Some probes return 0xffff_ffff because they can't read the pin back; not a failure.
    if interface
        .swj_pins(Pins(0), n_reset, Duration::ZERO)
        .is_err()
    {
        return false;
    }

    thread::sleep(Duration::from_millis(hold_ms));
    let _ = interface.swj_pins(n_reset, n_reset, Duration::ZERO);
    true
}

pub(super) fn retry_until_deadline(
    timeout: Duration,
    poll_interval: Duration,
    mut attempt: impl FnMut() -> Result<bool, ArmError>,
) -> Result<bool, ArmError> {
    let deadline = Instant::now() + timeout;
    loop {
        if attempt()? {
            return Ok(true);
        }
        if Instant::now() >= deadline {
            return Ok(false);
        }
        if !poll_interval.is_zero() {
            thread::sleep(poll_interval);
        }
    }
}

/// ADIv6 §B4.3.4 selection alert, the same for JTAG and SWD.
const DORMANT_ALERT_LO: u64 = 0x8685_2D95_6209_F392;
const DORMANT_ALERT_HI: u64 = 0x19BC_0EA2_E3DD_AFE9;

#[derive(Clone, Copy, Debug)]
enum DormantTarget {
    Jtag,
    Swd,
}

pub(super) fn enter_dormant(interface: &mut dyn DebugPortWire) {
    let Some(protocol) = interface.active_protocol() else {
        return;
    };

    let entered = interface
        .swj_sequence(&BitSequence::from_u64(51, 0x0007_FFFF_FFFF_FFFF))
        .is_ok()
        && match protocol {
            WireProtocol::Swd => interface
                .swj_sequence(&BitSequence::from_u64(16, 0xE3BC))
                .is_ok(),
            WireProtocol::Jtag => interface
                .swj_sequence(&BitSequence::from_u64(31, 0x33BB_BBBA))
                .is_ok(),
        };

    if entered {
        tracing::debug!("Infineon SWJ-DP: entered dormant state from {protocol:?}");
    } else {
        tracing::debug!("Infineon SWJ-DP: failed to enter dormant state from {protocol:?}");
    }
}

/// The caller must still confirm that the DP selected `target`.
fn send_dormant_alert(interface: &mut dyn DebugPortWire, target: DormantTarget) -> bool {
    let line_reset = interface
        .swj_sequence(&BitSequence::from_u64(51, 0x0007_FFFF_FFFF_FFFF))
        .is_ok();
    let protocol_to_dormant = line_reset
        && match target {
            DormantTarget::Jtag => interface
                .swj_sequence(&BitSequence::from_u64(16, 0xE3BC))
                .is_ok(), // SWD → dormant
            DormantTarget::Swd => interface
                .swj_sequence(&BitSequence::from_u64(31, 0x33BB_BBBA))
                .is_ok(), // JTAG → dormant
        };
    let framed = line_reset && protocol_to_dormant;

    let preamble = framed
        && interface
            .swj_sequence(&BitSequence::from_u64(8, 0xFF))
            .is_ok();
    let alert_lo = preamble
        && interface
            .swj_sequence(&BitSequence::from_u64(64, DORMANT_ALERT_LO))
            .is_ok();
    let alert_hi = alert_lo
        && interface
            .swj_sequence(&BitSequence::from_u64(64, DORMANT_ALERT_HI))
            .is_ok();
    let alerted = preamble && alert_lo && alert_hi;

    // 4 low cycles + activation code, then the trailer to idle.
    let activated = alerted
        && match target {
            DormantTarget::Jtag => {
                interface.swj_sequence(&BitSequence::from_u64(12, 0x0A0)).is_ok()               // JTAG activation 0x0A
                    && interface.swj_sequence(&BitSequence::from_u64(6, 0x3F)).is_ok()          // Test-Logic-Reset
                    && interface.jtag_sequence(false, &BitSequence::from_u64(1, 0x01)).is_ok() // Run-Test/Idle
            }
            DormantTarget::Swd => {
                interface.swj_sequence(&BitSequence::from_u64(12, 0x1A0)).is_ok()                       // SWD activation 0x1A
                    && interface.swj_sequence(&BitSequence::from_u64(51, 0x0007_FFFF_FFFF_FFFF)).is_ok() // line reset
                    && interface.swj_sequence(&BitSequence::from_u64(3, 0x00)).is_ok() // idle
            }
        };

    tracing::debug!(
        "Infineon SWJ-DP: dormant alert target={target:?} line_reset={line_reset} protocol_to_dormant={protocol_to_dormant} preamble={preamble} alert_lo={alert_lo} alert_hi={alert_hi} activated={activated}"
    );
    activated
}

/// IDCODE scan: the length scan fails on chains with a non-bypassing TAP after reset.
pub(super) fn jtag_tap_visible(interface: &mut dyn DebugPortWire) -> bool {
    let Some(mut chain) = interface.try_jtag_chain() else {
        return interface.configure_jtag(false).is_ok();
    };
    chain.set_chain(&[]);
    let scanned = match chain.scan_chain() {
        Ok(scanned) if !scanned.is_empty() => scanned.to_vec(),
        _ => return false,
    };
    chain.set_expected(&scanned);
    interface.configure_jtag(true).is_ok()
}

pub(super) fn try_jtag_dormant_wake(
    interface: &mut dyn DebugPortWire,
    timeout: Option<Duration>,
) -> Result<bool, ArmError> {
    let timeout = timeout.unwrap_or(Duration::from_millis(RESET_FINISH_DELAY_MS));
    let mut attempt = 0u32;
    tracing::debug!(
        "Infineon SWJ-DP: starting dormant-to-JTAG wake, timeout={}ms",
        timeout.as_millis()
    );
    let woken = retry_until_deadline(timeout, Duration::from_millis(5), || {
        attempt += 1;
        if send_dormant_alert(interface, DormantTarget::Jtag) {
            if jtag_tap_visible(interface) {
                if let Some(mut chain) = interface.try_jtag_chain()
                    && let Err(error) = chain.select(0)
                {
                    tracing::debug!(
                        "Infineon SWJ-DP: JTAG wake attempt {attempt}: selecting TAP 0 failed: {error}"
                    );
                    return Ok(false);
                }
                tracing::debug!("DORMANT-to-JTAG wake succeeded (JTAG TAP visible)");
                return Ok(true);
            }
            tracing::debug!(
                "Infineon SWJ-DP: JTAG wake attempt {attempt}: post-wake scan found no TAPs"
            );
        } else {
            tracing::debug!("Infineon SWJ-DP: JTAG wake attempt {attempt}: dormant alert failed");
        }
        Ok(false)
    })?;

    if !woken {
        tracing::warn!(
            "DORMANT-to-JTAG wake did not produce a TAP within {}ms ({} attempts)",
            timeout.as_millis(),
            attempt
        );
    }
    Ok(woken)
}

/// A failed `DPIDR` read returns the DP to dormant, so every attempt re-sends the alert.
pub(super) fn try_swd_dormant_connect(
    interface: &mut dyn DebugPortWire,
    timeout: Duration,
) -> Result<bool, ArmError> {
    let mut attempt = 0u32;
    let connected = retry_until_deadline(timeout, Duration::from_millis(5), || {
        attempt += 1;
        if send_dormant_alert(interface, DormantTarget::Swd) {
            match interface.raw_read_register(
                Port::Dp,
                RegisterAddress::DpRegister(DPIDR::ADDRESS).a2_and_3(),
            ) {
                Ok(v) => {
                    tracing::debug!("DORMANT-to-SWD connect: attempt {attempt}: DPIDR=0x{v:08X}");
                    return Ok(true);
                }
                Err(e) => {
                    tracing::trace!("DORMANT-to-SWD connect: attempt {attempt}: DPIDR NACK: {e}");
                }
            }
        }
        Ok(false)
    })?;

    if !connected {
        tracing::warn!("DORMANT-to-SWD connect timed out after {attempt} attempts");
    }
    Ok(connected)
}

/// Falls back to the default setup so that a real error surfaces.
pub(super) fn jtag_dormant_wake_or_default(
    interface: &mut dyn DebugPortWire,
    dp: DpAddress,
    timeout: Option<Duration>,
) -> Result<(), ArmError> {
    if try_jtag_dormant_wake(interface, timeout)? {
        return DefaultArmSequence(()).debug_port_connect(interface, dp);
    }
    DefaultArmSequence(()).debug_port_setup(interface, dp)
}

/// The SWJ-DP powers up dormant, where the stock JTAG-to-SWD switch does nothing.
pub(super) fn swd_dormant_connect_or_default(
    interface: &mut dyn DebugPortWire,
    dp: DpAddress,
    timeout: Option<Duration>,
) -> Result<(), ArmError> {
    let timeout = timeout.unwrap_or(Duration::from_millis(RESET_FINISH_DELAY_MS));
    if try_swd_dormant_connect(interface, timeout)? {
        return DefaultArmSequence(()).debug_port_connect(interface, dp);
    }
    DefaultArmSequence(()).debug_port_setup(interface, dp)
}

/// Best-effort DP recovery after a failed AHB access.
pub(super) fn recover_dp(interface: &mut dyn DapAccess, dp: DpAddress) {
    let mut abort = Abort(0);
    abort.set_dapabort(true);
    abort.set_orunerrclr(true);
    abort.set_wderrclr(true);
    abort.set_stkerrclr(true);
    abort.set_stkcmpclr(true);
    match interface.write_raw_dp_register(dp, Abort::ADDRESS, abort.0) {
        Ok(()) => tracing::debug!("Cleared DP sticky error flags"),
        Err(e) => tracing::warn!(
            "Failed to clear DP sticky error flags: {:?} (potential debug transport issue)",
            e
        ),
    }
}

/// `AIRCR.SYSRESETREQ` reset for targets whose system reset also resets the debug port.
pub(super) fn cortex_m_reset_system_with_recovery(
    interface: &mut dyn ArmMemoryInterface,
    recover: impl FnMut(&mut dyn ArmMemoryInterface) -> Result<(), ArmError>,
) -> Result<(), ArmError> {
    let mut aircr = Aircr(0);
    aircr.vectkey();
    aircr.set_sysresetreq(true);

    if let Err(err) = interface.write_word_32(Aircr::get_mmio_address(), aircr.into()) {
        // The reset can drop the debug port before the AIRCR write is acknowledged.
        if !is_no_acknowledge(&err) {
            return Err(err);
        }
        tracing::debug!("AIRCR write was not acknowledged, assuming the target reset: {err}");
    }

    cortex_m_wait_for_reset_with_recovery(interface, recover)
}

fn is_no_acknowledge(err: &ArmError) -> bool {
    let mut cause: &(dyn Error + 'static) = err;
    loop {
        if matches!(
            cause.downcast_ref::<ArmError>(),
            Some(ArmError::Dap(DapError::NoAcknowledge))
        ) {
            return true;
        }
        let Some(next) = cause.source() else {
            return false;
        };
        cause = next;
    }
}

pub(super) fn cortex_m_wait_for_reset_with_recovery(
    interface: &mut dyn ArmMemoryInterface,
    mut recover: impl FnMut(&mut dyn ArmMemoryInterface) -> Result<(), ArmError>,
) -> Result<(), ArmError> {
    let start = Instant::now();

    // PSOC 6 documentation: the debug port is back at most 600 ms after reset.
    while start.elapsed() < Duration::from_millis(600) {
        let dhcsr = match interface.read_word_32(Dhcsr::get_mmio_address()) {
            Ok(val) => Dhcsr(val),
            Err(err) => {
                if is_no_acknowledge(&err) {
                    if let Ok(probe) = interface.get_arm_debug_interface() {
                        // Let the reset propagate before reopening the debug connection.
                        thread::sleep(Duration::from_millis(100));
                        probe.reinitialize()?;
                        recover(interface)?;
                    }
                    continue;
                }

                return Err(err);
            }
        };
        if !dhcsr.s_reset_st() {
            return Ok(());
        }
    }

    Err(ArmError::Timeout)
}
