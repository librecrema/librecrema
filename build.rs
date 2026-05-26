use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let package_name = env::var("CARGO_PKG_NAME").unwrap();

    // Dynamically locate the target directory by traversing up from OUT_DIR.
    // OUT_DIR is guaranteed to be: target/<target-triple>/<profile>/build/<package-hash>/out
    // Moving up 3 levels brings us to target/<target-triple>/<profile>/
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target_dir = out_dir
        .parent() // target/.../<profile>/build/<package-hash>
        .and_then(|p| p.parent()) // target/.../<profile>/build
        .and_then(|p| p.parent()) // target/.../<profile>
        .unwrap_or_else(|| {
            panic!("Failed to find the target profile directory from OUT_DIR");
        });

    let output_file = target_dir.join(format!("{}.h", package_name));

    // Configure and run cbindgen
    cbindgen::Builder::new()
        .with_crate(crate_dir)
        .with_language(cbindgen::Language::C)
        .with_no_includes()
        .with_sys_include("stdint.h")
        .with_sys_include("stdbool.h")
        .generate()
        .map_or_else(
            |err| {
                // Fail soft during cargo test if cbindgen is missing in a clean target environment
                println!("cargo:warning=cbindgen failed to generate headers: {}", err);
            },
            |bindings| {
                bindings.write_to_file(output_file);
            },
        );

    // Watch the updated safe core architecture files
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/framework.rs");
    println!("cargo:rerun-if-changed=src/nodes.rs");
    println!("cargo:rerun-if-changed=src/ffi.rs");
}
