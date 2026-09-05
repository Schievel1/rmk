use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::{env, fs};

use const_gen::*;
use xz2::read::XzEncoder;

fn main() {
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
    let memory_x = if bin_name == "peripheral" {
        "memory-peripheral.x"
    } else {
        "memory-central.x"
    };
    // Re-run the build script when a different binary is built — otherwise
    // Cargo caches the output and the wrong memory.x is linked.
    println!("cargo:rerun-if-env-changed=RMK_DFU_BIN");

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let memory_content = fs::read(manifest_dir.join(memory_x)).unwrap();
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(&memory_content)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory-central.x");
    println!("cargo:rerun-if-changed=memory-peripheral.x");

    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");
}

fn generate_vial_config() {
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
    let const_declarations = {
        let a = const_declaration!(pub VIAL_KEYBOARD_DEF = keyboard_def_compressed);
        let b = const_declaration!(pub VIAL_KEYBOARD_ID = keyboard_id);
        format!("{}\n{}", a, b)
    };
    fs::write(out_file, const_declarations).unwrap();
}
