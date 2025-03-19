use std::fmt::Write;

pub struct MirrorStruct {
    pub jph_name: &'static str,
    pub jpc_name: &'static str,
    pub fields: &'static [StructField],
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
        for StructField { ty, name } in self.fields {
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
        for StructField { ty, name } in self.fields {
            writeln!(out, "\toutJpc->{name} = inJph->m{name};")?;
        }
        writeln!(out, "}}")?;
        writeln!(out)?;

        writeln!(
            out,
            "JPC_IMPL void {jpc_name}_to_jph(const {jpc_name}* inJpc, {jph_name}* outJph) {{"
        )?;
        for StructField { ty, name } in self.fields {
            writeln!(out, "\toutJph->m{name} = inJpc->{name};")?;
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

pub struct StructField {
    pub ty: &'static str,
    pub name: &'static str,
}
