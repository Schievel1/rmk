//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.
//!
//! The build script also sets the linker flags to tell it which link script to use.

use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::{env, fs};

use const_gen::*;
use xz2::read::XzEncoder;

fn main() {
    // Generate vial config at the root of project
    println!("cargo:rerun-if-changed=vial.json");
    generate_vial_config();

    // The central (dfu_ext) and the peripheral (internal DFU) use different
    // flash layouts, so pick the matching memory.x per binary. Cargo does not
    // expose the bin name to build scripts, so the Makefile sets RMK_DFU_BIN;
    // the central layout is the default for plain `cargo build`.
    let bin_name = match env::var("RMK_DFU_BIN") {
        Ok(bin) => bin,
        Err(_) => {
            println!(
                "cargo:warning=RMK_DFU_BIN not set - linking ALL bins with central's memory.x. The peripheral binary must be built with its own layout (e.g. `cargo make bin-peripheral`)."
            );
            "central".to_string()
        }
    };
    let memory_x = if bin_name == "central" {
        "memory-central.x"
    } else {
        "memory-peripheral.x"
    };
    // Re-run the build script when a different binary is built — otherwise
    // Cargo caches the output and the wrong memory.x is linked.
    println!("cargo:rerun-if-env-changed=RMK_DFU_BIN");

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let memory_content = fs::read(manifest_dir.join(memory_x)).unwrap();
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(&memory_content)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying the memory
    // files here, we ensure the build script is only re-run when
    // they change.
    println!("cargo:rerun-if-changed=memory-central.x");
    println!("cargo:rerun-if-changed=memory-peripheral.x");

    // Specify linker arguments.

    // `--nmagic` is required if memory section addresses are not aligned to 0x10000,
    // for example the FLASH and RAM sections in your `memory.x`.
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg=--nmagic");

    // Set the linker script to the one provided by cortex-m-rt.
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Set the linker script of the defmt
    println!("cargo:rustc-link-arg=-Tdefmt.x");
}

fn generate_vial_config() {
    // Generated vial config file
    let out_file = Path::new(&env::var_os("OUT_DIR").unwrap()).join("config_generated.rs");

    let p = Path::new("vial.json");
    let mut content = String::new();
    match File::open(p) {
        Ok(mut file) => {
            file.read_to_string(&mut content).expect("Cannot read vial.json");
        }
        Err(e) => println!("Cannot find vial.json {:?}: {}", p, e),
    };

    let vial_cfg = json::stringify(json::parse(&content).unwrap());
    let mut keyboard_def_compressed: Vec<u8> = Vec::new();
    XzEncoder::new(vial_cfg.as_bytes(), 6)
        .read_to_end(&mut keyboard_def_compressed)
        .unwrap();

    let keyboard_id: Vec<u8> = vec![0xB9, 0xBC, 0x09, 0xB2, 0x9D, 0x37, 0x4C, 0xEA];
    let const_declarations = [
        const_declaration!(pub VIAL_KEYBOARD_DEF = keyboard_def_compressed),
        const_declaration!(pub VIAL_KEYBOARD_ID = keyboard_id),
    ]
    .map(|s| "#[allow(clippy::redundant_static_lifetimes)]\n".to_owned() + s.as_str())
    .join("\n");
    fs::write(out_file, const_declarations).unwrap();
}
