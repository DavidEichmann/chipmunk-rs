extern crate cmake;

fn main() {
    // Builds the project in the directory located in `chipmunk`, installing it
    // into $OUT_DIR
    let dst = cmake::Config::new("chipmunk")
            .define("BUILD_DEMOS", "OFF")
            .define("INSTALL_DEMOS", "OFF")
            .define("BUILD_SHARED", "OFF")
            .build();

    println!("cargo:rustc-link-search=native={}", dst.join("lib").display());
    println!("cargo:rustc-link-lib=static=chipmunk");
}
