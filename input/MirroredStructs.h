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