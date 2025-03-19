macro_rules! builtin_types {
    (
        $($builtin:ident)*
    ) => {
        [$(stringify!($builtin),)*]
    }
}

#[cps::cps]
macro_rules! include_builtin_types {
    () =>
    let $($body:tt)* = cps::include!("input/BuiltinTypes.inc") in
    {
        builtin_types!($($body)*)
    }
}

macro_rules! mirrored_structs {
    (
        $(struct $struct_name:ident {
            $(
                $([[$field_attr:tt]])*
                $field_ty:ident
                $field_name:ident;
            )*
        };)*
    ) => {
        [$(
            MirrorStruct {
                jph_name: concat!("JPH::", stringify!($struct_name)),
                jpc_name: concat!("JPC_", stringify!($struct_name)),
                fields: &[$(
                    mirrored_struct_field!($([[$field_attr]])* $field_ty $field_name),
                )*],
            },
        )*]
    };
}

macro_rules! mirrored_struct_field {
    ($field_ty:ident $field_name:ident) => {
        StructField {
            ty: stringify!($field_ty),
            name: stringify!($field_name),
            is_superclass: false,
        }
    };

    ([[superclass]] $field_ty:ident $field_name:ident) => {
        StructField {
            ty: stringify!($field_ty),
            name: stringify!($field_name),
            is_superclass: true,
        }
    };
}

#[cps::cps]
macro_rules! include_mirrored_structs {
    () =>
    let $($body:tt)* = cps::include!("input/MirroredStructs.h") in
    {
        mirrored_structs!($($body)*)
    }
}

macro_rules! mirrored_enums {
    (
        $(enum $enum_name:ident : $repr:ident {
            $($member_name:ident $(= $member_value:expr)?,)*
        };)*
    ) => {

        vec![$(
            #[allow(unused_variables, unused_assignments)]
            {
                let mut counter = 0;

                MirrorEnum {
                    jph_name: concat!("JPH::E", stringify!($enum_name)),
                    jpc_name: concat!("JPC_", stringify!($enum_name)),
                    repr: stringify!($repr),
                    members: vec![$(
                        EnumMember {
                            jph_name: format!("JPH::E{}::{}", stringify!($enum_name), stringify!($member_name)),
                            jpc_name: format!("JPC_{}_{}",
                                heck::AsShoutySnakeCase(stringify!($enum_name)),
                                heck::AsShoutySnakeCase(stringify!($member_name))),
                            value: enum_value!(counter, $($member_value)?),
                        },
                    )*],
                }
            },
        )*]
    }
}

macro_rules! enum_value {
    ($counter:ident, $member_value:expr) => {{
        $counter = $member_value + 1;
        stringify!($member_value).to_owned()
    }};

    ($counter:ident,) => {{
        let val = $counter.to_string();
        $counter += 1;
        val
    }};
}

#[cps::cps]
macro_rules! include_mirrored_enums {
    () =>
    let $($body:tt)* = cps::include!("input/Enums.h") in
    {
        mirrored_enums!($($body)*)
    }
}
