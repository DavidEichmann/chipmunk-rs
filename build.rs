extern crate cmake;

use std::env::var;
use std::str::FromStr;

const DEBUG_DEFAULT: bool = false;

fn main() {
    // Builds the project in the directory located in `chipmunk`, installing it
    // into $OUT_DIR
    let debug: bool = var("DEBUG")
        .map(|s| bool::from_str(&s).unwrap_or(DEBUG_DEFAULT))
        .unwrap_or(DEBUG_DEFAULT);

    let dst = cmake::Config::new("chipmunk")
        .define("BUILD_DEMOS", "OFF")
        .define("INSTALL_DEMOS", "OFF")
        .define("BUILD_SHARED", "OFF")
        .define("CMAKE_BUILD_TYPE", if debug { "Debug" } else { "Release" })
        .build();

    println!("cargo:rustc-link-search=native={}",
             dst.join("lib").display());
    println!("cargo:rustc-link-lib=static=chipmunk");
}
