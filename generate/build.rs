use std::env::var;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(var("OUT_DIR").unwrap());

    let mut input_dir = PathBuf::from(env!("CARGO_WORKSPACE_DIR"));
    input_dir.push("input");

    let builtin = fs_err::read_to_string(input_dir.join("BuiltinTypes.inc")).unwrap();
    let builtin_out =
        "macro_rules! all_builtin_types { () => { builtin_types!(".to_owned() + &builtin + ") }}";
    fs_err::write(out_dir.join("builtin.rs"), builtin_out).unwrap();

    let enums = fs_err::read_to_string(input_dir.join("Enums.h")).unwrap();
    let enums_out =
        "macro_rules! all_enums { () => { mirrored_enums!(".to_owned() + &enums + ") }}";
    fs_err::write(out_dir.join("enums.rs"), enums_out).unwrap();

    let structs = fs_err::read_to_string(input_dir.join("MirroredStructs.h")).unwrap();
    let structs_out = "macro_rules! all_mirrored_structs { () => { mirrored_structs!(".to_owned()
        + &structs
        + ") }}";
    fs_err::write(out_dir.join("mirrored_structs.rs"), structs_out).unwrap();

    for entry in fs_err::read_dir(input_dir).unwrap() {
        let entry = entry.unwrap();
        println!("cargo:rerun-if-changed={}", entry.path().display());
    }
}
