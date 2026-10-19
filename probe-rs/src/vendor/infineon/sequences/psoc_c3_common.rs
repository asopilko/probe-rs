//! Shared primitives for the Infineon PSOC C3 debug sequences.
//!
//! The `PsocC3` (M3/M5/P2/P5) and `PsocC3X7X8`
//! (P7/P8/M7/M8) sequences share the same proprietary SYS AP bus-access portal and
//! the same TrustZone-aware CM33 AP `CSW` convention (`HNONSEC` derived from
//! `SDeviceEn`). This module holds the register definitions and helpers common to
//! all of them so they are defined once.

use std::{thread, time::Duration};

use bitfield::bitfield;

use crate::{
    MemoryMappedRegister,
    architecture::arm::{
        ApV2Address, ArmDebugInterface, ArmError, DapAccess, FullyQualifiedApAddress,
        core::armv7m::Dhcsr,
        dp::{Ctrl, DpAddress, DpRegister},
        memory::ArmMemoryInterface,
        sequences::ArmDebugSequence,
        traits::DebugPortWire,
    },
    probe::WireProtocol,
    vendor::DefaultArmSequence,
};

use super::common;
pub(super) use super::common::AP_CSW;
use super::common::{
    SYS_AP_BASE, cortex_m_wait_for_reset_with_recovery, jtag_dp_powerup, try_jtag_dormant_wake,
};

const XRES_HOLD_MS: u64 = 100;

/// Spans the whole boot: the boot ROM enables the SWJ pins only part-way through it.
const DORMANT_WAKE_TIMEOUT_MS: u64 = 1300;
const RESET_DELAY_MS: u64 = 400;
const POST_RESET_SETTLE_MS: u64 = 10;

/// The SWD-to-JTAG switch and the dormant alert both put an awake JTAG DP back to dormant.
pub(super) fn debug_port_setup_dormant_jtag(
    interface: &mut dyn DebugPortWire,
    dp: DpAddress,
) -> Result<(), ArmError> {
    if interface.active_protocol() != Some(WireProtocol::Jtag) {
        return common::swd_dormant_connect_or_default(interface, dp, None);
    }

    let tap_is_visible = common::jtag_tap_visible(interface);

    tracing::debug!("PSOC C3: initial JTAG TAP visible={tap_is_visible}");

    if tap_is_visible {
        if let Some(mut chain) = interface.try_jtag_chain()
            && let Err(error) = chain.select(0)
        {
            return Err(ArmError::Other(format!(
                "PSOC C3: selecting JTAG TAP 0 failed: {error}"
            )));
        }
        return Ok(());
    }

    // `debug_port_stop` leaves the DP dormant; try a wake before resetting with XRES.
    tracing::debug!("PSOC C3: initial JTAG TAP missing; trying dormant wake without XRES");
    if common::try_jtag_dormant_wake(
        interface,
        Some(Duration::from_millis(DORMANT_WAKE_TIMEOUT_MS)),
    )? {
        return DefaultArmSequence(()).debug_port_connect(interface, dp);
    }

    // XRES restarts the boot ROM, which re-enables the SWJ pins.
    tracing::warn!("PSOC C3: cannot attach without a reset; resetting the target via XRES");
    pulse_xres(interface);
    let result = common::jtag_dormant_wake_or_default(
        interface,
        dp,
        Some(Duration::from_millis(DORMANT_WAKE_TIMEOUT_MS)),
    );
    tracing::debug!(
        "PSOC C3: JTAG dormant recovery result={:?}",
        result.as_ref().err()
    );
    result
}

pub(super) fn debug_port_setup(
    interface: &mut dyn DebugPortWire,
    dp: DpAddress,
) -> Result<(), ArmError> {
    if interface.active_protocol() == Some(WireProtocol::Jtag) {
        tracing::debug!("PSOC C3: JTAG — dormant-to-JTAG wake with XRES reset-acquire");
        return self::debug_port_setup_dormant_jtag(interface, dp);
    }

    // Phase 1: try to connect without resetting (400 ms = `__Reset_Finish_Delay`).
    // Handles fresh power-on, post-soft-reset reconnect, and stale SWD sessions.
    tracing::debug!("PSOC C3: Phase 1 — non-destructive dormant connect (400 ms)");
    if common::try_swd_dormant_connect(interface, Duration::from_millis(RESET_DELAY_MS))? {
        tracing::debug!("PSOC C3: Phase 1 connected");
        return DefaultArmSequence(()).debug_port_connect(interface, dp);
    }
    // Phase 2: BootROM window has closed (user code running). Pulse XRES to
    // restart BootROM and re-open the window, then retry for 1.2 s.
    tracing::warn!("PSOC C3: cannot attach without a reset; resetting the target via XRES");
    match interface.target_reset() {
        Ok(()) => tracing::debug!("PSOC C3: Phase 2 — XRES pulsed"),
        Err(e) => tracing::warn!("PSOC C3: Phase 2 — XRES unavailable: {e}"),
    }
    thread::sleep(Duration::from_millis(10));

    tracing::debug!("PSOC C3: Phase 2 — dormant connect loop (1.2 s)");
    if common::try_swd_dormant_connect(interface, Duration::from_millis(1200))? {
        tracing::debug!("PSOC C3: Phase 2 connected");
        return DefaultArmSequence(()).debug_port_connect(interface, dp);
    }

    tracing::warn!("PSOC C3: all connection attempts failed");
    DefaultArmSequence(()).debug_port_connect(interface, dp)
}

pub(super) fn debug_port_start(
    interface: &mut dyn DapAccess,
    dp: DpAddress,
) -> Result<(), ArmError> {
    common::dp_start_with_powerup(interface, dp, true, true)
}

pub(super) fn debug_port_stop(
    interface: &mut dyn DebugPortWire,
    dp: DpAddress,
) -> Result<(), ArmError> {
    let result = DefaultArmSequence(()).debug_port_stop(interface, dp);
    common::enter_dormant(interface);
    result
}

fn pulse_xres(interface: &mut dyn DebugPortWire) {
    if common::pulse_xres_for(interface, XRES_HOLD_MS) {
        tracing::debug!("PSOC C3: XRES pulsed");
    } else {
        tracing::warn!("PSOC C3: probe cannot control nRESET; XRES pulse skipped");
    }
}

bitfield! {
    /// SYS AP CSW register — Infineon proprietary bus-access AP.
    ///
    /// Bit layout matches the standard AMBA AHB-AP CSW layout (ADI spec C2.2).
    #[derive(Clone, Copy)]
    pub(super) struct SysApCsw(u32);
    impl Debug;

    /// DbgSwEnable — must be 1 to enable DAP software access.
    pub dbg_sw_enable, set_dbg_sw_enable: 31;
    /// HNONSEC — 0 = secure transaction, 1 = non-secure.
    pub hnonsec, set_hnonsec: 30;
    /// MasterType — selects debug master ID on HMASTER signals.
    pub master_type, set_master_type: 29;
    /// HPROT[3] Cacheable.
    pub cacheable, set_cacheable: 27;
    /// HPROT[1] Privileged.
    pub privileged, set_privileged: 25;
    /// HPROT[0] Data.
    pub data, set_data: 24;
    /// Size[2:0] — access width: 0=byte, 1=halfword, 2=word.
    u8, size, set_size: 2, 0;
}

impl SysApCsw {
    /// Standard value for SRSS writes:
    /// DbgSwEnable=1, HNONSEC=0, MasterType=1, Cacheable=1, Privileged=1, Data=1, Size=Word.
    pub(super) fn secure_word() -> Self {
        let mut v = Self(0);
        v.set_dbg_sw_enable(true);
        v.set_hnonsec(false); // secure
        v.set_master_type(true);
        v.set_cacheable(true);
        v.set_privileged(true);
        v.set_data(true);
        v.set_size(2); // Word
        v
    }
}

bitfield! {
    /// CM33 AP CSW register (TrustZone-aware AHB5 AP).
    #[derive(Clone, Copy)]
    pub(super) struct Cm33ApCsw(u32);
    impl Debug;

    /// CSW.Size\[2:0\] — access size: 0=byte, 1=halfword, 2=word.
    u8, size, set_size: 2, 0;
    /// CSW.AddrInc\[5:4\] — address increment: 0=off, 1=single, 2=packed.
    u8, addr_inc, set_addr_inc: 5, 4;
    /// CSW.DeviceEn bit — AP is open when set.
    pub device_en, _: 6;
    /// CSW.SDeviceEn/SPIDEN — secure debug enabled when set.
    pub s_device_en, _: 23;
    /// CSW.Data (HPROT\[0\]) — data access.
    pub data, set_data: 24;
    /// CSW.Privileged (HPROT\[1\]) — privileged access.
    pub privileged, set_privileged: 25;
    /// CSW.Cacheable (HPROT\[3\]) — cacheable access.
    pub cacheable, set_cacheable: 27;
    /// CSW.HNONSEC bit — non-secure transaction when set.
    pub hnonsec, set_hnonsec: 30;
}

impl Cm33ApCsw {
    /// Bits preserved from the original CSW (implementation-defined/reserved).
    /// Clears Size[2:0], AddrInc[5:4], and HPROT[0,1,3] for explicit reconfiguration.
    const PRESERVE_MASK: u32 = 0xB0FF_FFC0;

    /// Build the standard CSW for word-width privileged debug access.
    ///
    /// Preserves implementation-defined bits, then explicitly sets Size=Word,
    /// AddrInc=Single, HPROT\[0,1,3\], and HNONSEC from SDeviceEn.
    pub(super) fn with_standard_access(csw: u32) -> Self {
        let mut out = Self(csw & Self::PRESERVE_MASK);
        out.set_size(2); // Word (32-bit)
        out.set_addr_inc(1); // Single increment
        out.set_data(true); // HPROT[0]: data access
        out.set_privileged(true); // HPROT[1]: privileged
        out.set_cacheable(true); // HPROT[3]: cacheable
        // HNONSEC=0 when SDeviceEn=1 (secure debug active), =1 otherwise.
        out.set_hnonsec((csw >> 23) & 1 == 0);
        out
    }
}

/// The SYS AP for the given debug port.
pub(super) fn sys_ap(dp: DpAddress) -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v2_with_dp(dp, ApV2Address(Some(SYS_AP_BASE)))
}

/// `FLASHC_FLASH_CTL` register — flash controller configuration (CM33 AP view).
pub(super) const FLASHC_FLASH_CTL: u64 = 0x5215_0000;
/// `FLASHC_FLASH_CTL.BANK` (bit 12) — set when the flash is in dual-bank mode.
pub(super) const FLASHC_FLASH_CTL_BANK: u32 = 0x0000_1000;

/// Physical flash bank layout as configured in `FLASHC_FLASH_CTL.BANK`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum FlashBankMode {
    /// The main flash is a single contiguous bank (`main0` only).
    Single,
    /// The main flash is split into two equal banks (`main0` + `main1`),
    /// each mapped at its own S-bus / C-bus alias.
    Dual,
}

/// Read `FLASHC_FLASH_CTL.BANK` over the given (CM33) AP to detect whether the
/// device is running in single- or dual-bank flash mode, and log the result.
///
/// The bank layout is a provisioning choice, not a property of the part number, so it can
/// only be learned from the live device. The result selects the erase strategy: a dual-bank
/// part has a second bank that the flash algorithms cannot reach, and is erased through the
/// SROM API instead (see [`super::psoc_c3_erase`]).
///
/// The read is best-effort — any transport error is returned to the caller, which may choose
/// to ignore it. Attach must not fail just because this register could not be read; treating
/// the device as single-bank keeps the stock behaviour.
pub(super) fn detect_flash_bank_mode(
    iface: &mut dyn ArmDebugInterface,
    ap: &FullyQualifiedApAddress,
) -> Result<FlashBankMode, ArmError> {
    let ctl = {
        let mut mem = iface.memory_interface(ap)?;
        mem.read_word_32(FLASHC_FLASH_CTL)?
    };

    let mode = if ctl & FLASHC_FLASH_CTL_BANK != 0 {
        FlashBankMode::Dual
    } else {
        FlashBankMode::Single
    };

    tracing::debug!(
        "PSOC C3: FLASHC_FLASH_CTL={:#010x} → {:?} flash bank mode",
        ctl,
        mode
    );

    Ok(mode)
}

/// CM33 access port base address, identical across every PSOC C3 family.
///
/// The `__apid` index the CMSIS pack assigns to this AP differs between families, but the
/// address it resolves to does not.
pub(super) const CM33_AP_BASE: u64 = 0xF000_2000;

/// The CM33 AP for the given debug port.
pub(super) fn cm33_ap(dp: DpAddress) -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v2_with_dp(dp, ApV2Address::new(CM33_AP_BASE))
}

/// SRSS soft reset control register (secure alias) — triggers a system soft reset.
///
/// For the M3/M5/P2/P5 family this is already the secure alias. The
/// x7/x8 family defines the non-secure base (`0x4220_0410`) and conditionally ORs in
/// the secure alias offset, which yields this same address.
pub(super) const SRSS_RES_SOFT_CTL: u32 = 0x5220_0410;
/// `SRSS_RES_SOFT_CTL.TRIG_SOFT` — triggers an immediate system reset.
pub(super) const SRSS_RES_SOFT_CTL_TRIG_SOFT: u32 = 1;

/// Apply the standard TrustZone-aware CM33 AP `CSW` and log the flash bank mode.
///
/// Reads the AP `CSW`, warns if the AP is closed (`DeviceEn == 0`, which typically
/// means a debug-certificate WFA unlock is required — not supported here), then
/// rewrites `CSW` with word-width privileged access and `HNONSEC` derived from
/// `SDeviceEn`. Finally performs a best-effort flash-bank-mode detection.
///
/// Shared by the generic `debug_device_unlock` and reused after a soft reset to
/// re-apply the `CSW` (the SRSS soft reset clears it).
pub(super) fn apply_standard_cm33_csw(
    interface: &mut dyn ArmDebugInterface,
    ap: &FullyQualifiedApAddress,
) -> Result<(), ArmError> {
    let csw = interface.read_raw_ap_register(ap, AP_CSW)?;
    let csw_bits = Cm33ApCsw(csw);

    if !csw_bits.device_en() {
        // The vendor unlock flow from here uploads a signed debug certificate
        // from the host and requests a WFA unlock — not implemented, since
        // probe-rs has no certificate-file infrastructure.
        tracing::warn!(
            "PSOC C3: CM33 AP is closed (DeviceEn=0), CSW: 0x{:08X}. \
             This chip may require a debug-certificate WFA unlock, which is not \
             supported yet — subsequent AP register accesses will likely fail.",
            csw
        );
    }

    let new_csw = Cm33ApCsw::with_standard_access(csw);

    tracing::debug!(
        "PSOC C3: AP CSW 0x{:08X} -> 0x{:08X} (SDeviceEn={}, HNONSEC={})",
        csw,
        new_csw.0,
        csw_bits.s_device_en(),
        new_csw.hnonsec(),
    );

    interface.write_raw_ap_register(ap, AP_CSW, new_csw.0)?;

    // Detect and log the flash bank mode (single vs dual). Best-effort — a read
    // failure here must not abort attach, so the error is intentionally ignored.
    let _ = detect_flash_bank_mode(interface, ap);

    Ok(())
}

/// Over JTAG the soft reset drops the TAP until the boot ROM re-enables the SWJ pins.
pub(super) fn reestablish_jtag_after_reset(interface: &mut dyn ArmMemoryInterface, dp: DpAddress) {
    let Ok(arm) = interface.get_arm_debug_interface() else {
        return;
    };
    if ArmDebugInterface::active_wire_protocol(arm) != Some(WireProtocol::Jtag) {
        return;
    }

    let _ = arm.debug_port_reconnect_with(&mut |probe| {
        // The dormant alert can disturb an already-awake TAP.
        if !common::jtag_tap_visible(probe) {
            let woken =
                try_jtag_dormant_wake(probe, Some(Duration::from_millis(DORMANT_WAKE_TIMEOUT_MS)))
                    .unwrap_or(false);
            if !woken {
                tracing::warn!("PSOC C3: JTAG TAP did not reappear after reset");
            }
        } else if let Some(mut chain) = probe.try_jtag_chain()
            && let Err(error) = chain.select(0)
        {
            tracing::warn!("PSOC C3: selecting JTAG TAP 0 after reset failed: {error}");
        }
        Ok(())
    });

    jtag_dp_powerup(arm, dp, true);
}

/// Lets the posted JTAG pipeline settle after the reset before core access resumes.
fn synchronize_jtag_after_reset(interface: &mut dyn ArmMemoryInterface, dp: DpAddress) {
    let Ok(arm) = interface.get_arm_debug_interface() else {
        return;
    };
    if ArmDebugInterface::active_wire_protocol(arm) != Some(WireProtocol::Jtag) {
        return;
    }

    let ctrl = arm.read_raw_dp_register(dp, Ctrl::ADDRESS);
    let cm33_ap = common::cm33(dp);
    let ap_idr = arm.read_raw_ap_register(&cm33_ap, 0xDFC);
    tracing::debug!(
        "PSOC C3: post-reset JTAG synchronization: CTRL/STAT={ctrl:?}, CM33 AP IDR={ap_idr:?}"
    );
    let _ = arm.flush();
    thread::sleep(Duration::from_millis(POST_RESET_SETTLE_MS));
}

pub(super) fn reconnect_after_reset(
    interface: &mut dyn ArmMemoryInterface,
    dp: DpAddress,
) -> Result<(), ArmError> {
    let arm = interface.get_arm_debug_interface()?;
    reconnect_debug_port(arm, dp)
}

pub(super) fn reconnect_debug_port(
    arm: &mut dyn ArmDebugInterface,
    dp: DpAddress,
) -> Result<(), ArmError> {
    arm.debug_port_reconnect_with(&mut |probe| {
        debug_port_setup_dormant_jtag(probe, dp)?;
        DefaultArmSequence(()).debug_port_connect(probe, dp)
    })?;
    common::dp_start_with_powerup(arm, dp, true, true)
}

/// Halt the CM33 and trigger an SRSS soft reset through the SYS AP, then wait for
/// the reset to complete.
///
/// The default `AIRCR.SYSRESETREQ` reset does not work on these devices; the vendor
/// CMSIS sequence uses `SRSS_RES_SOFT_CTL` via the SYS AP, which works regardless of
/// CM33 AP state. The core is halted first so that user code holding the AHB bus
/// cannot stall the SYS AP write and hang the AHB fabric until a power cycle.
///
/// On return the reset has completed but the CM33 AP `CSW` has been cleared by the
/// reset; callers must re-apply it (see [`apply_standard_cm33_csw`]).
pub(super) fn halt_and_soft_reset(
    interface: &mut dyn ArmMemoryInterface,
    delay: Duration,
) -> Result<(), ArmError> {
    // Halt the CM33 before writing SRSS. If user code holds the AHB bus the
    // SYS AP write stalls and permanently hangs the AHB fabric until power cycle.
    let mut dhcsr = Dhcsr(0);
    dhcsr.set_c_debugen(true);
    dhcsr.set_c_halt(true);
    dhcsr.enable_write();
    let _ = interface.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into());

    let dp = interface.fully_qualified_address().dp();
    let sys_ap = sys_ap(dp);

    {
        let arm = interface.get_arm_debug_interface()?;
        arm.write_raw_ap_register(&sys_ap, AP_CSW, SysApCsw::secure_word().0)?;
        // Write triggers an immediate soft reset; transaction error is expected.
        let _ = common::write_mem32(arm, &sys_ap, SRSS_RES_SOFT_CTL, SRSS_RES_SOFT_CTL_TRIG_SOFT);
        // Flush the batch so the DRW write reaches the chip before we sleep —
        // otherwise the write can be deferred until the first post-reset read,
        // which then races the reset itself.
        let _ = arm.flush();
    }

    tracing::debug!(
        "PSOC C3: reset_system — waiting {}ms for boot ROM",
        delay.as_millis()
    );
    thread::sleep(delay);

    reestablish_jtag_after_reset(interface, dp);
    synchronize_jtag_after_reset(interface, dp);

    cortex_m_wait_for_reset_with_recovery(interface, |interface| {
        reconnect_after_reset(interface, dp)
    })?;

    Ok(())
}
