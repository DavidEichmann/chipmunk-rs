
//! Provides raw ffi bindings to the Chipmunk2D library.

use libc::{c_double, c_int, c_uint};

#[repr(C)]
#[derive(Clone,Copy,PartialEq,Debug)]
/// A Chipmunk2D vector.
pub struct CPVect {
    /// The x component.
    pub x: CPFloat,

    /// The y component.
    pub y: CPFloat,
}

impl CPVect {
    /// Create a new CPVect.
    pub fn new(x: f64, y: f64) -> CPVect {
        CPVect { x: x, y: y }
    }
}

/// A Chipmunk2D floating point value.
pub type CPFloat = c_double;

/// A Chipmunk2D Space.
pub enum CPSpace {}

/// A Chipmunk2D Body.
pub enum CPBody {}

/// A Chipmunk2D Shape.
pub enum CPShape {}

/// A Chipmunk2D CircleShape.
pub enum CPCircleShape {}

/// A Chipmunk2D PolyShape.
pub enum CPPolyShape {}

/// A Chipmunk2D Constraint.
pub enum CPConstraint {}

/// A Chipmunk2D Pin Joint
pub enum CPPinJoint {}

/// A Chipmunk2D Slide Joint
pub enum CPSlideJoint {}

/// A Chipmunk2D Constraint.
pub enum CPPivotJoint {}

/// A Chipmunk2D Groove Joint
pub enum CPGrooveJoint {}

/// A Chipmunk2D Damped Spring
pub enum CPDampedSpring {}

/// A Chipmunk2D DampedRotary Spring
pub enum CPDampedRotarySpring {}

/// A Chipmunk2D RotaryLimit Joint
pub enum CPRotaryLimitJoint {}

/// A Chipmunk2D Ratchet Joint
pub enum CPRatchetJoint {}

/// A Chipmunk2D Gear Joint
pub enum CPGearJoint {}

/// A Chipmunk2D Simple Motor
pub enum CPSimpleMotor {}


extern "C" {
    // Space
    pub fn cpSpaceNew() -> *const CPSpace;
    pub fn cpSpaceFree(CPSpace: *const CPSpace);

    // Space Settings
    pub fn cpSpaceIsLocked(space: *const CPSpace) -> bool;
    pub fn cpSpaceGetGravity(CPSpace: *const CPSpace) -> CPVect;
    pub fn cpSpaceGetCurrentTimeStep(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetIterations(space: *const CPSpace) -> c_int;
    pub fn cpSpaceGetDamping(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetIdleSpeedThreshold(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetSleepTimeThreshold(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetCollisionSlop(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetCollisionBias(space: *const CPSpace) -> CPFloat;
    pub fn cpSpaceGetCollisionPersistence(space: *const CPSpace) -> c_uint;
    pub fn cpSpaceGetStaticBody(space: *const CPSpace) -> *const CPBody;
    pub fn cpSpaceSetGravity(CPSpace: *const CPSpace, gravity: CPVect);
    pub fn cpSpaceSetIterations(space: *const CPSpace, value: c_int);
    pub fn cpSpaceSetDamping(space: *const CPSpace, value: CPFloat);
    pub fn cpSpaceSetIdleSpeedThreshold(space: *const CPSpace, value: CPFloat);
    pub fn cpSpaceSetSleepTimeThreshold(space: *const CPSpace, value: CPFloat);
    pub fn cpSpaceSetCollisionSlop(space: *const CPSpace, value: CPFloat);
    pub fn cpSpaceSetCollisionBias(space: *const CPSpace, value: CPFloat);
    pub fn cpSpaceSetCollisionPersistence(space: *const CPSpace, value: c_uint);

    // Space Operations
    pub fn cpSpaceStep(space: *const CPSpace, dt: CPFloat);
    pub fn cpSpaceAddShape(space: *const CPSpace, shape: *const CPShape) -> *const CPShape;
    pub fn cpSpaceAddBody(space: *const CPSpace, body: *const CPBody) -> *const CPBody;
    pub fn cpSpaceRemoveShape(space: *const CPSpace, shape: *const CPShape);
    pub fn cpSpaceRemoveBody(space: *const CPSpace, body: *const CPBody);
    pub fn cpSpaceContainsShape(space: *const CPSpace, shape: *const CPShape) -> bool;
    pub fn cpSpaceContainsBody(space: *const CPSpace, body: *const CPBody) -> bool;
    pub fn cpSpaceAddConstraint(space: *const CPSpace, constraint: *const CPConstraint);
    pub fn cpSpaceRemoveConstraint(space: *const CPSpace, constraint: *const CPConstraint);
    pub fn cpSpaceContainsConstraint(space: *const CPSpace, constraint: *const CPConstraint) -> bool;

    pub fn cpSpaceReindexShape(space: *const CPSpace, shape: *const CPShape);
    pub fn cpSpaceReindexShapesForBody(space: *const CPSpace, body: *const CPBody);
    pub fn cpSpaceReindexStatic(space: *const CPSpace);

    // TODO iterators for body/shape/Constraint

    // Constraint
    pub fn cpConstraintFree(constraint: *const CPConstraint);
    // cpBody * cpConstraintGetA(constraint: *const CPConstraint)
    // cpBody * cpConstraintGetB(constraint: *const CPConstraint)
    // cpSpace* cpConstraintGetSpace(constraint: *const CPConstraint)
    pub fn cpConstraintGetMaxForce(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpConstraintGetErrorBias(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpConstraintGetMaxBias(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpConstraintGetCollideBodies(constraint: *const CPConstraint) -> bool;
    pub fn cpConstraintSetMaxForce(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpConstraintSetErrorBias(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpConstraintSetMaxBias(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpConstraintSetCollideBodies(constraint: *const CPConstraint, collideBodies: bool);

    // Pin Joints
    pub fn cpPinJointAlloc() -> *const CPPinJoint;
    pub fn cpPinJointInit(joint: *const CPPinJoint, a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect) -> *const CPPinJoint;
    pub fn cpPinJointNew(a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect) -> *const CPConstraint;

    pub fn cpPinJointGetanchorA(constraint: *const CPConstraint) -> CPVect;
    pub fn cpPinJointSetanchorA(constraint: *const CPConstraint, value: CPVect);
    pub fn cpPinJointGetanchorB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpPinJointSetanchorB(constraint: *const CPConstraint, value: CPVect);
    pub fn cpPinJointGetDist(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpPinJointSetDist(constraint: *const CPConstraint, value: CPFloat);

    // Slide Joints
    pub fn cpSlideJointAlloc() -> *const CPSlideJoint;
    pub fn cpSlideJointInit(joint: *const CPSlideJoint, a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect, min: CPFloat, max: CPFloat) -> *const CPSlideJoint;
    pub fn cpSlideJointNew(a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect, min: CPFloat, max: CPFloat) -> *const CPConstraint;

    pub fn cpSlideJointGetanchorA(constraint: *const CPConstraint) -> CPVect;
    pub fn cpSlideJointSetanchorA(constraint: *const CPConstraint, value: CPVect);
    pub fn cpSlideJointGetanchorB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpSlideJointSetanchorB(constraint: *const CPConstraint, value: CPVect);
    pub fn cpSlideJointGetMin(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpSlideJointSetMin(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpSlideJointGetMax(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpSlideJointSetMax(constraint: *const CPConstraint, value: CPFloat);

    // Pivot Joints
    pub fn cpPivotJointAlloc() -> *const CPPivotJoint;
    pub fn cpPivotJointInit(joint: *const CPPivotJoint, a: *const CPBody, b: *const CPBody, pivot: CPVect) -> *const CPPivotJoint;
    pub fn cpPivotJointNew(a: *const CPBody, b: *const CPBody, pivot: CPVect) -> *const CPConstraint;
    pub fn cpPivotJointNew2(a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect) -> *const CPConstraint;

    pub fn cpPivotJointGetanchorA(constraint: *const CPConstraint) -> CPVect;
    pub fn cpPivotJointSetanchorA(constraint: *const CPConstraint, value: CPVect);
    pub fn cpPivotJointGetanchorB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpPivotJointSetanchorB(constraint: *const CPConstraint, value: CPVect);

    // Groove Joint
    pub fn cpGrooveJointAlloc() -> *const CPGrooveJoint;
    pub fn cpGrooveJointInit(joint: *const CPGrooveJoint, a: *const CPBody, b: *const CPBody, groove_a: CPVect, groove_b: CPVect, anchorB: CPVect) -> *const CPGrooveJoint;
    pub fn cpGrooveJointNew(a: *const CPBody, b: *const CPBody, groove_a: CPVect, groove_b: CPVect, anchorB: CPVect) -> *const CPConstraint;

    pub fn cpGrooveJointGetGrooveA(constraint: *const CPConstraint) -> CPVect;
    pub fn cpGrooveJointSetGrooveA(constraint: *const CPConstraint, value: CPVect);
    pub fn cpGrooveJointGetGrooveB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpGrooveJointSetGrooveB(constraint: *const CPConstraint, value: CPVect);
    pub fn cpGrooveJointGetanchorB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpGrooveJointSetanchorB(constraint: *const CPConstraint, value: CPVect);

    // Damped Spring
    pub fn cpDampedSpringAlloc() -> *const CPDampedSpring;
    pub fn cpDampedSpringInit(joint: *const CPDampedSpring, a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect, restLength: CPFloat, stiffness: CPFloat, damping: CPFloat) -> *const CPDampedSpring;
    pub fn cpDampedSpringNew(a: *const CPBody, b: *const CPBody, anchorA: CPVect, anchorB: CPVect, restLength: CPFloat, stiffness: CPFloat, damping: CPFloat) -> *const CPConstraint;
    
    pub fn cpDampedSpringGetanchorA(constraint: *const CPConstraint) -> CPVect;
    pub fn cpDampedSpringSetanchorA(constraint: *const CPConstraint, value: CPVect);
    pub fn cpDampedSpringGetanchorB(constraint: *const CPConstraint) -> CPVect;
    pub fn cpDampedSpringSetanchorB(constraint: *const CPConstraint, value: CPVect);
    pub fn cpDampedSpringGetRestLength(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedSpringSetRestLength(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpDampedSpringGetStiffness(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedSpringSetStiffness(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpDampedSpringGetDamping(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedSpringSetDamping(constraint: *const CPConstraint, value: CPFloat);

    // Damped Rotary Spring
    pub fn cpDampedRotarySpringAlloc() -> *const CPDampedRotarySpring;
    pub fn cpDampedRotarySpringInit(joint: *const CPDampedRotarySpring, a: *const CPBody, b: *const CPBody, restAngle: CPFloat, stiffness: CPFloat, damping: CPFloat) -> *const CPDampedRotarySpring;
    pub fn cpDampedRotarySpringNew(a: *const CPBody, b: *const CPBody, restAngle: CPFloat, stiffness: CPFloat, damping: CPFloat) -> *const CPConstraint;

    pub fn cpDampedRotarySpringGetRestAngle(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedRotarySpringSetRestAngle(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpDampedRotarySpringGetStiffness(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedRotarySpringSetStiffness(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpDampedRotarySpringGetDamping(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpDampedRotarySpringSetDamping(constraint: *const CPConstraint, value: CPFloat);

    // Rotary Limit Joint
    pub fn cpRotaryLimitJointAlloc() -> *const CPRotaryLimitJoint;
    pub fn cpRotaryLimitJointInit(joint: *const CPRotaryLimitJoint, a: *const CPBody, b: *const CPBody, min: CPFloat, max: CPFloat) -> *const CPRotaryLimitJoint;
    pub fn cpRotaryLimitJointNew(a: *const CPBody, b: *const CPBody, min: CPFloat, max: CPFloat) -> *const CPConstraint;

    pub fn cpRotaryLimitJointGetMin(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpRotaryLimitJointSetMin(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpRotaryLimitJointGetMax(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpRotaryLimitJointSetMax(constraint: *const CPConstraint, value: CPFloat);

    // Ratchet Joint
    pub fn cpRatchetJointAlloc() -> *const CPRatchetJoint;
    pub fn cpRatchetJointInit(joint: *const CPRatchetJoint, a: *const CPBody, b: *const CPBody, phase: CPFloat, ratchet: CPFloat) -> *const CPRatchetJoint;
    pub fn cpRatchetJointNew(a: *const CPBody, b: *const CPBody, phase: CPFloat, ratchet: CPFloat) -> *const CPConstraint;

    pub fn cpRatchetJointGetAngle(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpRatchetJointSetAngle(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpRatchetJointGetPhase(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpRatchetJointSetPhase(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpRatchetJointGetRatchet(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpRatchetJointSetRatchet(constraint: *const CPConstraint, value: CPFloat);

    // Gear Joint
    pub fn cpGearJointAlloc() -> *const CPGearJoint;
    pub fn cpGearJointInit(joint: *const CPGearJoint, a: *const CPBody, b: *const CPBody, phase: CPFloat, ratio: CPFloat) -> *const CPGearJoint;
    pub fn cpGearJointNew(a: *const CPBody, b: *const CPBody, phase: CPFloat, ratio: CPFloat) -> *const CPConstraint;

    pub fn cpGearJointGetPhase(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpGearJointSetPhase(constraint: *const CPConstraint, value: CPFloat);
    pub fn cpGearJointGetRatio(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpGearJointSetRatio(constraint: *const CPConstraint, value: CPFloat);

    // Simple Motor
    pub fn cpSimpleMotorAlloc() -> *const CPSimpleMotor;
    pub fn cpSimpleMotorInit(joint: *const CPSimpleMotor, a: *const CPBody, b: *const CPBody, rate: CPFloat) -> *const CPSimpleMotor;
    pub fn cpSimpleMotorNew(a: *const CPBody, b: *const CPBody, rate: CPFloat) -> *const CPConstraint;

    pub fn cpSimpleMotorGetRate(constraint: *const CPConstraint) -> CPFloat;
    pub fn cpSimpleMotorSetRate(constraint: *const CPConstraint, value: CPFloat);

    // Body
    pub fn cpBodyNew(m: CPFloat, i: CPFloat) -> *const CPBody;
    pub fn cpBodyNewStatic() -> *const CPBody;
    pub fn cpBodyNewKinematic() -> *const CPBody;
    pub fn cpBodyFree(body: *const CPBody);

    pub fn cpBodyGetMass(body: *const CPBody) -> CPFloat;
    pub fn cpBodySetMass(body: *const CPBody, m: CPFloat);
    pub fn cpBodyGetMoment(body: *const CPBody) -> CPFloat;
    pub fn cpBodySetMoment(body: *const CPBody, i: CPFloat);
    pub fn cpBodyGetPosition(body: *const CPBody) -> CPVect;
    pub fn cpBodySetPosition(body: *const CPBody, pos: CPVect);
    pub fn cpBodyGetCenterOfGravity(body: *const CPBody) -> CPVect;
    pub fn cpBodySetCenterOfGravity(body: *const CPBody, cog: CPVect);
    pub fn cpBodyGetVelocity(body: *const CPBody) -> CPVect;
    pub fn cpBodySetVelocity(body: *const CPBody, value: CPVect);
    pub fn cpBodyGetForce(body: *const CPBody) -> CPVect;
    pub fn cpBodySetForce(body: *const CPBody, value: CPVect);
    pub fn cpBodyGetAngle(body: *const CPBody) -> CPFloat;
    pub fn cpBodySetAngle(body: *const CPBody, a: CPFloat);
    pub fn cpBodyGetAngularVelocity(body: *const CPBody) -> CPFloat;
    pub fn cpBodySetAngularVelocity(body: *const CPBody, value: CPFloat);
    pub fn cpBodyGetTorque(body: *const CPBody) -> CPFloat;
    pub fn cpBodySetTorque(body: *const CPBody, value: CPFloat);
    pub fn cpBodyGetRotation(body: *const CPBody) -> CPVect;
    pub fn cpBodyGetSpace(body: *const CPBody) -> *const CPSpace; // May be null

    // Shape
    pub fn cpShapeFree(shape: *const CPShape);

    pub fn cpShapeGetSpace(shape: *const CPShape) -> *const CPSpace; // May be null
    pub fn cpShapeGetBody(shape: *const CPShape) -> *const CPBody;

    pub fn cpShapeSetBody(shape: *const CPShape, body: *const CPBody);
    pub fn cpShapeGetElasticity(shape: *const CPShape) -> CPFloat;
    pub fn cpShapeSetElasticity(shape: *const CPShape, value: CPFloat);
    pub fn cpShapeGetFriction(shape: *const CPShape) -> CPFloat;
    pub fn cpShapeSetFriction(shape: *const CPShape, value: CPFloat);
    // pub fn cpShapeGetBB(shape: *const CPShape) -> CPBB;
    // pub fn cpShapeGetSensor(shape: *const CPShape) -> CPBool;
    // pub fn cpShapeSetSensor(shape: *const CPShape, value: cpBool);
    // pub fn cpShapeGetSurfaceVelocity(shape: *const CPShape) -> CPVect;
    // pub fn cpShapeSetSurfaceVelocity(shape: *const CPShape, value: CPVect);
    // pub fn cpShapeGetCollisionType(shape: *const CPShape) -> CPCollisionType;
    // pub fn cpShapeSetCollisionType(shape: *const CPShape, value: cpCollisionType);
    // pub fn cpShapeGetFilter(shape: *const CPShape) -> CPShapeFilter;
    // pub fn cpShapeSetFilter(shape: *const CPShape, filter: cpShapeFilter);

    // Circle Shape
    pub fn cpCircleShapeNew(body: *const CPBody,
                            radius: CPFloat,
                            offset: CPVect)
                            -> *const CPCircleShape;
    pub fn cpCircleShapeGetOffset(circleShape: *const CPCircleShape) -> CPVect;
    pub fn cpCircleShapeGetRadius(circleShape: *const CPCircleShape) -> CPFloat;

    // Segment Shape
    pub fn cpSegmentShapeFree(shape: *const CPShape);
    pub fn cpSegmentShapeNew(body: *const CPBody,
                             a: CPVect,
                             b: CPVect,
                             radius: CPFloat)
                             -> *const CPShape;
    pub fn cpSegmentShapeGetA(shape: *const CPShape) -> CPVect;
    pub fn cpSegmentShapeGetB(shape: *const CPShape) -> CPVect;
    pub fn cpSegmentShapeGetNormal(shape: *const CPShape) -> CPVect;
    pub fn cpSegmentShapeGetRadius(shape: *const CPShape) -> CPFloat;
    pub fn cpSegmentShapeSetNeighbors(shape: *const CPShape, prev: CPVect, next: CPVect);

    // Polygon Shape
    // pub fn cpPolyShapeNew(body: *const CPBody, numVerts: c_int, verts: *const CPVect, transform: cpTransform, radius: CPFloat) -> *const CPPolyShape;
    pub fn cpPolyShapeNewRaw(body: *const CPBody,
                             numVerts: c_int,
                             verts: *const CPVect,
                             radius: CPFloat)
                             -> *const CPPolyShape;
    pub fn cpPolyShapeGetCount(shape: *const CPPolyShape) -> c_int;
    pub fn cpPolyShapeGetVert(shape: *const CPPolyShape, index: c_int) -> CPVect;
    pub fn cpPolyShapeGetRadius(shape: *const CPPolyShape) -> CPFloat;
    //pub fn cpCentroidForPoly(count: c_int, verts: *const CPVect) -> CPVect;

    // Box Shape
    pub fn cpBoxShapeNew(body: *const CPBody,
                         width: CPFloat,
                         height: CPFloat,
                         radius: CPFloat)
                         -> *const CPPolyShape;
}
