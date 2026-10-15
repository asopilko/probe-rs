use std::time::{Duration, Instant};

use super::{ResumeAction, RuntimeTarget};
use probe_rs_rpc::core_ops::{WireCoreStatus, WireSteppingMode};

use gdbstub::target::ext::base::multithread::MultiThreadSingleStepOps;
use gdbstub::target::ext::base::multithread::{MultiThreadResume, MultiThreadSingleStep};

/// How long a resumed core may still read as halted (#3965).
const RESUME_SETTLE_TIMEOUT: Duration = Duration::from_millis(100);

impl MultiThreadResume for RuntimeTarget {
    fn resume(&mut self) -> Result<(), Self::Error> {
        self.running_cores.clear();

        match self.resume_action {
            (_, ResumeAction::Resume) => {
                let cores: Vec<u32> = self.cores.iter().map(|core| core.index as u32).collect();
                let mut statuses = self
                    .block_on(self.session.resume_cores(Some(cores.clone())))?
                    .statuses;

                let start = Instant::now();
                // Right after run(), a core can still read as halted, even with its previous reason.
                while statuses
                    .iter()
                    .any(|(_, status)| matches!(status, WireCoreStatus::Halted(_)))
                    && start.elapsed() < RESUME_SETTLE_TIMEOUT
                {
                    std::thread::sleep(Duration::from_millis(1));
                    statuses = self
                        .block_on(self.session.cores_status(Some(cores.clone())))?
                        .statuses;
                }

                self.running_cores = cores.into_iter().map(|index| index as usize).collect();
            }
            (core_id, ResumeAction::Step) => {
                self.block_on(
                    self.session
                        .debug_step(core_id as u32, WireSteppingMode::StepInstruction),
                )?;
                self.running_cores.push(core_id);
            }
            (_, ResumeAction::Unchanged) => {}
        }

        Ok(())
    }

    fn clear_resume_actions(&mut self) -> Result<(), Self::Error> {
        self.resume_action = (0, ResumeAction::Resume);
        Ok(())
    }

    fn set_resume_action_continue(
        &mut self,
        tid: gdbstub::common::Tid,
        _signal: Option<gdbstub::common::Signal>,
    ) -> Result<(), Self::Error> {
        let core_id = tid.get() - 1;
        self.resume_action = (core_id, ResumeAction::Resume);
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<MultiThreadSingleStepOps<'_, Self>> {
        Some(self)
    }
}

impl MultiThreadSingleStep for RuntimeTarget {
    fn set_resume_action_step(
        &mut self,
        tid: gdbstub::common::Tid,
        _signal: Option<gdbstub::common::Signal>,
    ) -> Result<(), Self::Error> {
        let core_id = tid.get() - 1;
        self.resume_action = (core_id, ResumeAction::Step);
        Ok(())
    }
}
