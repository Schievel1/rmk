#[path = "./build_common.rs"]
mod common;

use std::process::Command;

fn main() {
    // Set the compilation target configuration
    let mut cfgs = common::CfgSet::new();
    common::set_target_cfgs(&mut cfgs);

    println!("cargo:rerun-if-changed=build.rs");

    // The git commit of this rmk checkout, empty for a copy without git history (crates.io).
    // Storage written by another commit is wiped, so the script reruns whenever HEAD moves.
    let git = |args: &[&str]| {
        Command::new("git")
            .args(args)
            .output()
            .ok()
            .filter(|out| out.status.success())
            .map(|out| String::from_utf8_lossy(&out.stdout).trim().to_string())
    };
    // The HEAD reflog grows on every HEAD move, exists in worktrees too, and is untouched by
    // `git fetch`/`gc`/`tag`. A branch's ref file is packed away by `gc` and missing in
    // worktrees, and cargo reruns the script on every build while a watched path is missing.
    if let Some(log) = git(&["rev-parse", "--git-path", "logs/HEAD"]) {
        println!("cargo:rerun-if-changed={log}");
    }
    let commit = git(&["rev-parse", "HEAD"]).unwrap_or_default();
    println!("cargo:rustc-env=RMK_COMMIT={commit}");

    // The enabled features, sorted: storage written under another feature set is wiped too.
    let mut features: Vec<String> = std::env::vars()
        .filter_map(|(key, _)| key.strip_prefix("CARGO_FEATURE_").map(str::to_lowercase))
        .collect();
    features.sort();
    println!("cargo:rustc-env=RMK_FEATURES={}", features.join(","));
}
