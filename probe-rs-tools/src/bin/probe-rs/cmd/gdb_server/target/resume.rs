use super::{ResumeAction, RuntimeTarget};
use probe_rs_rpc::core_ops::WireSteppingMode;

use gdbstub::target::ext::base::multithread::{
    MultiThreadResume, MultiThreadSchedulerLocking, MultiThreadSchedulerLockingOps,
    MultiThreadSingleStep, MultiThreadSingleStepOps,
};

impl MultiThreadResume for RuntimeTarget {
    fn resume(&mut self) -> Result<(), Self::Error> {
        let mut resume = Vec::new();
        for core in &self.cores {
            match self.resume_actions.get(&core.index) {
                Some(ResumeAction::Step) => {
                    self.block_on(
                        self.session
                            .debug_step(core.index as u32, WireSteppingMode::StepInstruction),
                    )?;
                }
                Some(ResumeAction::Resume) => resume.push(core.index as u32),
                None if !self.scheduler_locked => resume.push(core.index as u32),
                None => {}
            }
        }

        if !resume.is_empty() {
            self.block_on(self.session.resume_cores(Some(resume)))?;
        }

        Ok(())
    }

    fn clear_resume_actions(&mut self) -> Result<(), Self::Error> {
        self.resume_actions.clear();
        self.scheduler_locked = false;
        Ok(())
    }

    fn set_resume_action_continue(
        &mut self,
        tid: gdbstub::common::Tid,
        _signal: Option<gdbstub::common::Signal>,
    ) -> Result<(), Self::Error> {
        let core_id = tid.get() - 1;
        self.resume_actions.insert(core_id, ResumeAction::Resume);
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<MultiThreadSingleStepOps<'_, Self>> {
        Some(self)
    }

    fn support_scheduler_locking(&mut self) -> Option<MultiThreadSchedulerLockingOps<'_, Self>> {
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
        self.resume_actions.insert(core_id, ResumeAction::Step);
        Ok(())
    }
}

impl MultiThreadSchedulerLocking for RuntimeTarget {
    fn set_resume_action_scheduler_lock(&mut self) -> Result<(), Self::Error> {
        self.scheduler_locked = true;
        Ok(())
    }
}
