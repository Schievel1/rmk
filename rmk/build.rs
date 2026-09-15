#[path = "./build_common.rs"]
mod common;

use std::path::Path;
use std::process::Command;
use std::{env, fs};

fn main() {
    // Set the compilation target configuration
    let mut cfgs = common::CfgSet::new();
    common::set_target_cfgs(&mut cfgs);

    println!("cargo:rerun-if-changed=build.rs");

    // The git commit of this rmk checkout, 0 for a copy without git history (crates.io).
    // Storage written by another commit is wiped, so the script reruns whenever HEAD moves.
    let git = |args: &[&str]| {
        Command::new("git")
            .args(args)
            .output()
            .ok()
            .filter(|out| out.status.success())
            .map(|out| String::from_utf8_lossy(&out.stdout).trim().to_string())
    };
    if let Some(dir) = git(&["rev-parse", "--git-dir"]) {
        println!("cargo:rerun-if-changed={dir}/HEAD");
        println!("cargo:rerun-if-changed={dir}/packed-refs");
        if let Some(head) = git(&["symbolic-ref", "-q", "HEAD"]) {
            println!("cargo:rerun-if-changed={dir}/{head}");
        }
    }
    let commit = git(&["rev-parse", "--short=8", "HEAD"])
        .and_then(|hash| u32::from_str_radix(&hash[..hash.len().min(8)], 16).ok())
        .unwrap_or(0);
    let constants = Path::new(&env::var("OUT_DIR").unwrap()).join("constants.rs");
    fs::write(
        constants,
        format!("pub(crate) const RMK_COMMIT: u32 = {commit:#010x};\n"),
    )
    .unwrap();
}
