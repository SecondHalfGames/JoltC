macro_rules! mirrored_structs {
    (
        $(struct $struct_name:ident {
            $($field_ty:ident $field_name:ident;)*
        };)*
    ) => {
        [$(
            MirrorStruct {
                jph_name: concat!("JPH::", stringify!($struct_name)),
                jpc_name: concat!("JPC_", stringify!($struct_name)),
                fields: &[$(
                    StructField {
                        ty: stringify!($field_ty),
                        name: stringify!($field_name),
                    },
                )*]
            }
        )*]
    }
}

#[cps::cps]
macro_rules! include_mirrored_structs {
    ($source:literal) =>
    let $($body:tt)* = cps::include!($source) in
    {
        mirrored_structs!($($body)*)
    }
}
