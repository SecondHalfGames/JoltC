use std::fmt::Write;

fn is_builtin_type(ty: &str) -> bool {
    crate::BUILTIN_TYPES.contains(&ty)
}

fn is_generated_type(ty: &str) -> bool {
    crate::structs().iter().any(|s| s.jpc_name == ty)
}

#[expect(unused)]
fn get_struct(ty: &str) -> Option<&'static MirrorStruct> {
    crate::structs().iter().find(|s| s.jpc_name == ty)
}

fn get_enum(ty: &str) -> Option<&'static MirrorEnum> {
    crate::enums().iter().find(|e| e.jpc_name == ty)
}

fn is_enum(ty: &str) -> bool {
    crate::enums().iter().any(|e| e.jpc_name == ty)
}

fn jph_to_jpc(
    out: &mut impl Write,
    jpc_ty: &str,
    in_place: &str,
    out_place: &str,
) -> anyhow::Result<()> {
    if is_builtin_type(jpc_ty) {
        writeln!(out, "\t{out_place} = {in_place};")?;
    } else if is_generated_type(jpc_ty) {
        writeln!(out, "\t{jpc_ty}_to_jpc(&{out_place}, &{in_place});")?;
    } else if is_enum(jpc_ty) {
        writeln!(out, "\t{out_place} = static_cast<{jpc_ty}>({in_place});")?;
    } else {
        writeln!(out, "\t{out_place} = to_jpc({in_place});")?;
    }

    Ok(())
}

fn jpc_to_jph(
    out: &mut impl Write,
    jpc_ty: &str,
    in_place: &str,
    out_place: &str,
) -> anyhow::Result<()> {
    if is_builtin_type(jpc_ty) {
        writeln!(out, "\t{out_place} = {in_place};")?;
    } else if is_generated_type(jpc_ty) {
        writeln!(out, "\t{jpc_ty}_to_jph(&{in_place}, &{out_place});")?;
    } else if let Some(e) = get_enum(jpc_ty) {
        let jph_ty = e.jph_name;
        writeln!(out, "\t{out_place} = static_cast<{jph_ty}>({in_place});")?;
    } else {
        writeln!(out, "\t{out_place} = to_jph({in_place});")?;
    }

    Ok(())
}

pub struct MirrorStruct {
    pub jph_name: &'static str,
    pub jpc_name: &'static str,
    pub fields: &'static [StructField],
}

pub struct StructField {
    pub ty: &'static str,
    pub name: &'static str,
    pub is_superclass: bool,
}

impl MirrorStruct {
    pub fn emit_header(&self, out: &mut impl Write) -> anyhow::Result<()> {
        self.header_definition(out)?;
        self.header_methods(out)?;
        Ok(())
    }

    fn header_methods(&self, out: &mut impl Write) -> anyhow::Result<()> {
        let jpc_name = self.jpc_name;

        writeln!(out, "JPC_API void {jpc_name}_default({jpc_name}* value);")?;
        writeln!(out)?;

        Ok(())
    }

    fn header_definition(&self, out: &mut impl Write) -> anyhow::Result<()> {
        let jpc_name = self.jpc_name;

        writeln!(out, "typedef struct {jpc_name} {{")?;
        for StructField { ty, name, .. } in self.fields {
            writeln!(out, "\t{ty} {name};")?;
        }
        writeln!(out, "}} {jpc_name};")?;
        writeln!(out)?;

        Ok(())
    }

    pub fn emit_impl(&self, out: &mut impl Write) -> anyhow::Result<()> {
        self.impl_methods(out)?;
        Ok(())
    }

    fn impl_methods(&self, out: &mut impl Write) -> anyhow::Result<()> {
        let Self {
            jpc_name, jph_name, ..
        } = self;

        writeln!(
            out,
            "JPC_IMPL void {jpc_name}_to_jpc({jpc_name}* outJpc, const {jph_name}* inJph) {{"
        )?;
        for StructField {
            ty,
            name,
            is_superclass,
        } in self.fields
        {
            if *is_superclass {
                jph_to_jpc(out, ty, "*inJph", &format!("outJpc->{name}"))?;
            } else {
                jph_to_jpc(
                    out,
                    ty,
                    &format!("inJph->m{name}"),
                    &format!("outJpc->{name}"),
                )?;
            }
        }
        writeln!(out, "}}")?;
        writeln!(out)?;

        writeln!(
            out,
            "JPC_IMPL void {jpc_name}_to_jph(const {jpc_name}* inJpc, {jph_name}* outJph) {{"
        )?;
        for StructField {
            ty,
            name,
            is_superclass,
        } in self.fields
        {
            if *is_superclass {
                jpc_to_jph(out, ty, &format!("inJpc->{name}"), "*outJph")?;
            } else {
                jpc_to_jph(
                    out,
                    ty,
                    &format!("inJpc->{name}"),
                    &format!("outJph->m{name}"),
                )?;
            }
        }
        writeln!(out, "}}")?;
        writeln!(out)?;

        writeln!(out, "JPC_API void {jpc_name}_default({jpc_name}* value) {{")?;
        writeln!(out, "\t{jph_name} defaultValue{{}};")?;
        writeln!(out, "\t{jpc_name}_to_jpc(value, &defaultValue);")?;
        writeln!(out, "}}")?;
        writeln!(out)?;

        Ok(())
    }
}

pub struct MirrorEnum {
    pub jpc_name: &'static str,
    pub jph_name: &'static str,
    pub repr: &'static str,
    pub members: Vec<EnumMember>,
}

pub struct EnumMember {
    pub jpc_name: String,
    pub jph_name: String,
    pub value: String,
}

impl MirrorEnum {
    pub fn emit_header(&self, out: &mut impl Write) -> anyhow::Result<()> {
        let Self {
            jpc_name,
            members,
            repr,
            ..
        } = self;

        writeln!(out, "typedef {repr} {jpc_name};")?;
        for member in members {
            let EnumMember {
                jpc_name: jpc_member,
                value,
                ..
            } = member;

            writeln!(out, "const {jpc_name} {jpc_member} = {value};")?;
        }
        writeln!(out)?;

        for member in members {
            let EnumMember {
                jph_name: jph_member,
                jpc_name: jpc_member,
                ..
            } = member;

            writeln!(out, "ENSURE_ENUM_EQ({jpc_member}, {jph_member})")?;
        }
        writeln!(out)?;

        Ok(())
    }
}
