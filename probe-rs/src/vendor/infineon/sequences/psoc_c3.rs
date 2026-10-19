//! Debug sequences for PSOC C3 (M3/M5/P2/P5) devices.
//!
//! These devices' CM33 access port is a TrustZone-aware AHB5 AP whose `CSW.HNONSEC`
//! bit must be derived from `CSW.SDeviceEn` (secure debug enabled). The generic AHB5
//! AP adapter does not know about this Infineon-specific convention, so without this
//! correction AP register writes (e.g. `DRW`, used to halt the CPU via `DHCSR` before
//! flashing) fail with "Target device did not respond to request".
//!
//! If the AP is closed (`CSW.DeviceEn == 0`), the vendor unlock flow uploads a signed
//! debug certificate and requests a WFA ("wait for authentication") unlock — this
//! requires a certificate file from the host toolchain that probe-rs has no
//! infrastructure for yet, so it is not implemented here. If `DeviceEn` is closed,
//! this sequence reports the condition clearly instead of silently failing on the
//! next AP register access.

use std::{sync::Arc, time::Duration};

use probe_rs_target::Chip;

use crate::{
    Permissions,
    architecture::arm::{
        ArmDebugInterface, ArmError, DapAccess, FullyQualifiedApAddress,
        dp::DpAddress,
        memory::ArmMemoryInterface,
        sequences::{ArmDebugSequence, DebugEraseSequence, DefaultArmSequence},
        traits::DebugPortWire,
    },
};

use super::common;
use super::{psoc_c3_common, psoc_c3_erase::DualBankErase};

/// Post-reset settle time — the boot ROM needs this long to re-open the AP after a
/// soft reset. PSC3M5 needs the same margin as x7/x8.
const RESET_DELAY_MS: u64 = common::RESET_FINISH_DELAY_MS;

/// Generic PSOC C3 debug sequence for devices without a custom debug-cert
/// unlock flow (M3/M5/P2/P5).
#[derive(Debug)]
pub struct PsocC3 {
    /// How this part has to be erased, decided once the device reports its bank layout.
    erase: DualBankErase,
}

impl PsocC3 {
    /// Creates the debug sequence for a generic PSOC C3 chip (M3/M5/P2/P5).
    pub fn create(chip: &Chip) -> Arc<Self> {
        Arc::new(PsocC3 {
            erase: DualBankErase::new(chip),
        })
    }
}

impl ArmDebugSequence for PsocC3 {
    /// The boot ROM must be running to wake the dormant SWJ-DP, so nRESET can't be held.
    fn reset_hardware_assert(&self, _interface: &mut dyn DebugPortWire) -> Result<(), ArmError> {
        tracing::warn!(
            "PSOC C3: `--connect-under-reset` is not supported; skipping nRESET assertion"
        );
        Ok(())
    }

    fn reset_hardware_deassert(
        &self,
        _interface: &mut dyn ArmDebugInterface,
        _default_ap: &FullyQualifiedApAddress,
    ) -> Result<(), ArmError> {
        Ok(())
    }

    fn debug_port_setup(
        &self,
        interface: &mut dyn DebugPortWire,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        psoc_c3_common::debug_port_setup(interface, dp)
    }

    fn debug_port_start(
        &self,
        interface: &mut dyn DapAccess,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        common::dp_start_with_powerup(interface, dp, true, true)
    }

    /// Corrects `CSW.HNONSEC` based on `CSW.SDeviceEn` on the default AP.
    ///
    /// Without this, AP register writes (e.g. `DRW`, used to write `DHCSR` when
    /// halting the CPU before flashing) fail with no response, because the generic
    /// AHB5 AP adapter does not apply Infineon's secure-debug-derived HNONSEC
    /// convention.
    fn debug_device_unlock(
        &self,
        interface: &mut dyn ArmDebugInterface,
        default_ap: &FullyQualifiedApAddress,
        _permissions: &Permissions,
    ) -> Result<(), ArmError> {
        psoc_c3_common::apply_standard_cm33_csw(interface, default_ap)?;

        // Record the flash bank layout for the erase path. Best-effort - a read failure here
        // must not abort attach, it only means a dual-bank part is treated as single-bank.
        if let Ok(mode) = psoc_c3_common::detect_flash_bank_mode(interface, default_ap) {
            self.erase.set_mode(mode);
        }

        Ok(())
    }

    fn debug_erase_sequence(&self) -> Option<Arc<dyn DebugEraseSequence>> {
        self.erase
            .sequence(psoc_c3_common::cm33_ap(DpAddress::Default))
    }

    /// Resets the SoC via `SRSS_RES_SOFT_CTL` (through the SYS AP) and re-applies the
    /// CM33 AP CSW correction afterwards.
    ///
    /// The default AIRCR.SYSRESETREQ-based reset doesn't work on this device — the
    /// vendor CMSIS sequence uses `SRSS_RES_SOFT_CTL` via the SYS AP instead, which
    /// works regardless of CM33 AP state. The SRSS soft reset also clears the CM33
    /// AP CSW (HNONSEC/SDeviceEn), so without re-applying it here, the very next AP
    /// register access (e.g. halting the core to run the flash algorithm) fails
    /// with "Target device did not respond to request".
    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: probe_rs_target::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        psoc_c3_common::halt_and_soft_reset(interface, Duration::from_millis(RESET_DELAY_MS))?;

        let cm33_ap = interface.fully_qualified_address();
        let arm = interface.get_arm_debug_interface()?;

        // SRSS soft reset clears the CM33 AP CSW — re-apply HNONSEC/access fields
        // so the very next AP register access doesn't fail.
        psoc_c3_common::apply_standard_cm33_csw(arm, &cm33_ap)
    }

    /// Sticky errors left by a reset or sleeping firmware would fault the stock DHCSR writes.
    fn debug_core_stop(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
    ) -> Result<(), ArmError> {
        let dp = interface.fully_qualified_address().dp();
        if let Ok(arm) = interface.get_arm_debug_interface()
            && let Err(e) = common::clear_dp_sticky_errors(arm, dp)
        {
            tracing::warn!("PSOC C3: failed to clear DP sticky errors before core stop: {e:?}");
        }

        self.debug_core_stop_default(interface, core_type)
    }

    fn debug_port_stop(
        &self,
        interface: &mut dyn DebugPortWire,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        let result = DefaultArmSequence(()).debug_port_stop(interface, dp);
        common::enter_dormant(interface);
        result
    }
}
