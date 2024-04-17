use std::fmt::Write;

use anyhow::Context;
use fs_err as fs;
use heck::AsShoutySnakeCase;
use regex::Regex;
use walkdir::WalkDir;

static PREFIX: &str = "JPC";

fn main() {
    if let Err(err) = generate() {
        eprintln!("Fatal error: {err:?}");
    }
}

fn generate() -> anyhow::Result<()> {
    let mut output = String::new();

    for entry in WalkDir::new("JoltPhysics/Jolt") {
        let entry = entry?;
        let file_name = entry
            .file_name()
            .to_str()
            .context("invalid UTF-8 in file name")?;

        if file_name.ends_with(".h") {
            println!("Processing {file_name}...");

            let header = fs::read_to_string(entry.path())?;

            convert_enums(&header, &mut output);
            convert_classes(&header, &mut output);
        }
    }

    eprintln!();
    eprintln!();
    eprintln!();
    eprintln!("{output}");

    Ok(())
}

fn convert_classes(header: &str, output: &mut String) {
    let class = Regex::new(r"^\s*class\s*JPH_EXPORT\s*(\w+)").unwrap();
    let field = Regex::new(r"^([\w<>])").unwrap();

    struct WipClass {
        cpp_name: String,
        c_name: String,
    }

    let mut current_class = None;

    for line in header.lines() {
        if current_class.is_none() {
            if let Some(captures) = class.captures(line) {
                let name = &captures[1];
                println!("Found exported class: {name}");

                current_class = Some(WipClass {
                    cpp_name: name.to_owned(),
                    c_name: format!("{PREFIX}_{name}"),
                });
            }

            continue;
        }

        if let Some(current) = &current_class {}
    }
}

fn convert_enums(header: &str, output: &mut String) {
    struct WipEnum {
        cpp_name: String,
        c_name: String,
    }

    let enum_class = Regex::new(r"^\s*enum\s+class\s+(\w+)(?:\s*:\s*(\w+))?").unwrap();
    let enum_member = Regex::new(r"^\s*(\w+)(?:\s*=\s*(.+?),)?").unwrap();
    let end_block = Regex::new(r"^\s*}").unwrap();

    let mut current_enum = None;

    for line in header.lines() {
        if current_enum.is_none() {
            if let Some(captures) = enum_class.captures(line) {
                let mut name = &captures[1];
                let repr = captures.get(2).map(|m| m.as_str());

                // EShapeSubType => ShapeSubType
                if name.starts_with('E') && name.as_bytes()[1].is_ascii_uppercase() {
                    name = &name[1..];
                }

                let c_name = format!("{PREFIX}_{}", AsShoutySnakeCase(name));

                if let Some(repr) = repr {
                    writeln!(output, "typedef enum {c_name} : {repr} {{").unwrap();
                } else {
                    writeln!(output, "typedef enum {c_name} {{").unwrap();
                }

                current_enum = Some(WipEnum {
                    cpp_name: name.to_owned(),
                    c_name,
                });

                continue;
            }

            continue;
        }

        if let Some(current) = &current_enum {
            if end_block.is_match(line) {
                writeln!(output, "}} {};", current.c_name).unwrap();
                current_enum = None;
                continue;
            }

            if let Some(captures) = enum_member.captures(line) {
                let name = &captures[1];
                let value = captures.get(2).map(|m| m.as_str());

                let c_name = format!("{}_{}", current.c_name, AsShoutySnakeCase(name));

                if let Some(value) = value {
                    writeln!(output, "    {} = {value},", c_name).unwrap();
                } else {
                    writeln!(output, "    {},", c_name).unwrap();
                }
            }
        }
    }
}
