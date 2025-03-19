use std::path::PathBuf;

fn main() {
    let mut input_dir = PathBuf::from(env!("CARGO_WORKSPACE_DIR"));
    input_dir.push("input");

    for entry in fs_err::read_dir(input_dir).unwrap() {
        let entry = entry.unwrap();
        println!("cargo:rerun-if-changed={}", entry.path().display());
    }
}
