#![cfg_attr(not(feature = "with-std"), no_std)]

pub mod framework;
pub mod nodes;
pub mod ffi;

// Statically allocated global scheduler instance
pub static mut GLOBAL_SCHEDULER: Option<framework::SafeScheduler> = Some(framework::SafeScheduler::new());

#[cfg(not(feature = "with-std"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}