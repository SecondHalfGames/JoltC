use std::io::Write;

use crate::PREFIX;

macro_rules! opaque_struct {
    ($c_name:ident : $cpp_name:literal {$(
        $c_ret:ident $cpp_meth:ident($(
            $c_ty:ident $c_arg:ident
        ),*) $(const)?;
    )*}) => {
        paste::paste! {
            OpaqueStruct {
                cpp_name: $cpp_name,
                c_name: stringify!($c_name),
                methods: vec![$(
                    WrapperMethod {
                        struct_c_name: stringify!($c_name),
                        cpp_name: stringify!($cpp_meth),
                        c_name: stringify!([<$c_name _ $cpp_meth>]),
                        c_ret: stringify!($c_ret),
                        self_const: stringify!()
                    },
                )*],
            }
        }
    };
}

pub fn define_api() -> Api {
    let body = opaque_struct!(JPC_Body : "JPH::Body" {
        JPC_BodyID GetID() const;
        JPC_BodyType GetBodyType() const;
        bool IsRigidBody() const;
        bool IsSoftBody() const;
        bool IsActive() const;
        bool IsState() const;
        bool IsKinematic() const;
        bool IsDynamic() const;
        bool CanBeKinematicOrDynamic() const;
        void SetIsSensor(bool inIsSensor);
        bool IsSensor() const;
        void SetCollideKinematicVsNonDynamic(bool inCollide);
        bool GetCollideKinematicVsNonDynamic() const;
        void SetUseManifoldReduction(bool inUseReduction);
        bool GetUseManifoldReduction() const;
        bool GetUseManifoldReductionWithBody(const Body &inBody2) const;
        void SetApplyGyroscopicForce(bool inApply);
        bool GetApplyGyroscopicForce() const;
        void SetEnhancedInternalEdgeRemoval(bool inApply);
        bool GetEnhancedInternalEdgeRemoval() const;
        bool GetEnhancedInternalEdgeRemovalWithBody(const Body &inBody2) const;
        JPC_MotionType GetMotionType() const;
        void SetMotionType(JPC_MotionType inMotionType);
        BroadPhaseLayer GetBroadPhaseLayer() const;
        ObjectLayer GetObjectLayer() const;
        const CollisionGroup & GetCollisionGroup() const;
        CollisionGroup & GetCollisionGroup();
        void SetCollisionGroup(const CollisionGroup &inGroup);
        bool GetAllowSleeping() const;
        void SetAllowSleeping(bool inAllow);
        void ResetSleepTimer();
        float GetFriction() const;
        void SetFriction(float inFriction);
        float GetRestitution() const;
        void SetRestitution(float inRestitution);
        Vec3 GetLinearVelocity() const;
        void SetLinearVelocity(JPC_Vec3 inLinearVelocity);
        void SetLinearVelocityClamped(JPC_Vec3 inLinearVelocity);
        Vec3 GetAngularVelocity() const;
        void SetAngularVelocity(JPC_Vec3 inAngularVelocity);
        void SetAngularVelocityClamped(JPC_Vec3 inAngularVelocity);
        Vec3 GetPointVelocityCOM(JPC_Vec3 inPointRelativeToCOM) const;
        Vec3 GetPointVelocity(JPC_RVec3 inPoint) const;
        void AddForce(JPC_Vec3 inForce);
        void AddForce(JPC_Vec3 inForce, JPC_RVec3 inPosition);
        void AddTorque(JPC_Vec3 inTorque);
        Vec3 GetAccumulatedForce() const;
        Vec3 GetAccumulatedTorque() const;
        void ResetForce();
        void ResetTorque();
        void ResetMotion();
        Mat44 GetInverseInertia() const;
        void AddImpulse(JPC_Vec3 inImpulse);
        void AddImpulse(JPC_Vec3 inImpulse, JPC_RVec3 inPosition);
        void AddAngularImpulse(JPC_Vec3 inAngularImpulse);
        void MoveKinematic(JPC_RVec3 inTargetPosition, QuatArg inTargetRotation, float inDeltaTime);
        bool ApplyBuoyancyImpulse(JPC_RVec3 inSurfacePosition, JPC_Vec3 inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, JPC_Vec3 inFluidVelocity, JPC_Vec3 inGravity, float inDeltaTime);
        bool IsInBroadPhase() const;
        bool IsCollisionCacheInvalid() const;
        const JPC_Shape* GetShape() const;
        RVec3 GetPosition() const;
        Quat GetRotation() const;
        RMat44 GetWorldTransform() const;
        RVec3 GetCenterOfMassPosition() const;
        RMat44 GetCenterOfMassTransform() const;
        RMat44 GetInverseCenterOfMassTransform() const;
        const AABox & GetWorldSpaceBounds() const;
        const MotionProperties *GetMotionProperties() const
        MotionProperties * GetMotionProperties();
        const MotionProperties *GetMotionPropertiesUnchecked() const
        MotionProperties * GetMotionPropertiesUnchecked();
        uint64_t GetUserData() const;
        void SetUserData(uint64_t inUserData);
        Vec3 GetWorldSpaceSurfaceNormal(const SubShapeID &inSubShapeID, JPC_RVec3 inPosition) const;
        TransformedShape GetTransformedShape() const;
        BodyCreationSettings GetBodyCreationSettings() const;
        SoftBodyCreationSettings GetSoftBodyCreationSettings() const;
    });

    Api {
        structs: vec![body],
    }
}

#[derive(Debug)]
pub struct Api {
    structs: Vec<OpaqueStruct>,
}

impl Api {
    pub fn header<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        for s in &self.structs {
            s.header(out)?;
        }

        Ok(())
    }

    pub fn implementation<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        for s in &self.structs {
            s.implementation(out)?;
        }

        Ok(())
    }
}

#[derive(Debug)]
struct OpaqueStruct {
    cpp_name: &'static str,
    c_name: &'static str,
    methods: Vec<WrapperMethod>,
}

impl OpaqueStruct {
    fn method(&mut self, method: impl Into<WrapperMethod>) {
        let mut method = method.into();
        method.struct_c_name = self.c_name;
        self.methods.push(method);
    }

    fn header<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        let c_name = self.c_name;

        writeln!(out, "typedef struct {c_name} {c_name};")?;

        for method in &self.methods {
            method.header(out)?;
        }

        Ok(())
    }

    fn implementation<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        let c_name = self.c_name;
        let cpp_name = self.cpp_name;

        writeln!(out, "OPAQUE_WRAPPER({c_name}, {cpp_name})");

        for method in &self.methods {
            method.implementation(out)?;
        }

        Ok(())
    }
}

#[derive(Debug)]
struct WrapperMethod {
    self_const: &'static str,
    struct_c_name: &'static str,
    cpp_name: &'static str,
    c_name: &'static str,
    c_ret: &'static str,
}

impl WrapperMethod {
    fn header<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        let self_const = self.self_const;
        let struct_c_name = self.struct_c_name;
        let c_name = self.c_name;
        let c_ret = self.c_ret;

        writeln!(
            out,
            "JPC_API {c_ret} {c_name}({self_const}{struct_c_name}* self);"
        )?;

        Ok(())
    }

    fn implementation<W: Write>(&self, out: &mut W) -> anyhow::Result<()> {
        let self_const = self.self_const;
        let struct_c_name = self.struct_c_name;
        let cpp_name = self.cpp_name;
        let c_name = self.c_name;
        let c_ret = self.c_ret;

        writeln!(
            out,
            "JPC_API {c_ret} {c_name}({self_const} {struct_c_name}* self) {{"
        )?;
        writeln!(out, "    return to_jpc(to_jph(self)->{cpp_name}());")?;
        writeln!(out, "}}")?;

        Ok(())
    }
}

impl From<&str> for WrapperMethod {
    fn from(value: &str) -> Self {
        todo!()
    }
}
