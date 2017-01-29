extern crate cmake;

fn main() {
    // Builds the project in the directory located in `chipmunk`, installing it
    // into $OUT_DIR
    let dst = cmake::build("chipmunk");

    println!("cargo:rustc-link-search=native={}", dst.display());
    println!("cargo:rustc-link-lib=static=chipmunk");
}
