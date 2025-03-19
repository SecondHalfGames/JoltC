enum ShapeType: uint8_t {
	Convex,
	Compound,
	Decorated,
	Mesh,
	HeightField,
	SoftBody,

	// User defined shapes
	User1,
	User2,
	User3,
	User4,

	Plane,
	Empty,
};

enum ShapeSubType: uint8_t {
	// Convex shapes
	Sphere,
	Box,
	Triangle,
	Capsule,
	TaperedCapsule,
	Cylinder,
	ConvexHull,

	// Compound shapes
	StaticCompound,
	MutableCompound,

	// Decorated shapes
	RotatedTranslated,
	Scaled,
	OffsetCenterOfMass,

	// Other shapes
	Mesh,
	HeightField,
	SoftBody,

	// User defined shapes
	User1,
	User2,
	User3,
	User4,
	User5,
	User6,
	User7,
	User8,

	// User defined convex shapes
	UserConvex1,
	UserConvex2,
	UserConvex3,
	UserConvex4,
	UserConvex5,
	UserConvex6,
	UserConvex7,
	UserConvex8,

	// Other shapes
	Plane,
	TaperedCylinder,
	Empty,
};

enum PhysicsUpdateError: uint32_t {
	None = 0,
	ManifoldCacheFull = 1 << 0,
	BodyPairCacheFull = 1 << 1,
	ContactConstraintsFull = 1 << 2,
};

enum ConstraintType: uint32_t {
	Constraint,
	TwoBodyConstraint,
};

enum ConstraintSubType: uint32_t {
	Fixed,
	Point,
	Hinge,
	Slider,
	Distance,
	Cone,
	SwingTwist,
	SixDOF,
	Path,
	Vehicle,
	RackAndPinion,
	Gear,
	Pulley,

	// User defined constraint types start here
	User1,
	User2,
	User3,
	User4,
};

enum ConstraintSpace: uint32_t {
	LocalToBodyCOM = 0,
	WorldSpace = 1,
};