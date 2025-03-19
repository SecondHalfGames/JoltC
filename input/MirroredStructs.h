// The structs in this file are C structs that are mirrored 1:1 with Jolt's C++
// types.

struct ConstraintSettings {
	bool Enabled;
	uint32_t ConstraintPriority;
	uint NumVelocityStepsOverride;
	uint NumPositionStepsOverride;
	float DrawConstraintSize;
	uint64_t UserData;
};

struct FixedConstraintSettings {
	[[superclass]] JPC_ConstraintSettings ConstraintSettings;

	// TwoBodyConstraintSettings: no extra members

	// FixedConstraintSettings
	JPC_ConstraintSpace Space;
	bool AutoDetectPoint;

	JPC_RVec3 Point1;
	JPC_Vec3 AxisX1;
	JPC_Vec3 AxisY1;

	JPC_RVec3 Point2;
	JPC_Vec3 AxisX2;
	JPC_Vec3 AxisY2;
};