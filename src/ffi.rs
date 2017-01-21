
//! Provides raw ffi bindings to the Chipmunk2D library.

use libc::{c_double, c_int, c_uint};

#[repr(C)]
#[derive(Clone,Copy)]
/// A Chipmink2D vector.
pub struct CPVect {
    /// The x component.
    pub x: CPFloat,

    /// The y component.
    pub y: CPFloat,
}

impl CPVect {
    /// Create a new CPVect.
    pub fn new(x: f64, y: f64) -> CPVect { CPVect { x: x, y: y } }
}

/// A Chipmink2D floating point value.
pub type CPFloat = c_double;

/// A Chipmink2D Space.
pub enum CPSpace {}

/// A Chipmink2D Body.
pub enum CPBody {}

/// A Chipmink2D Shape.
pub enum CPShape {}

/// A Chipmink2D CircleShape.
pub enum CPCircleShape {}

/// A Chipmink2D PolyShape.
pub enum CPPolyShape {}

/// A Chipmink2D Constraint.
pub enum CPConstraint {}

#[link(name="chipmunk")]
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
    pub fn cpSpaceAddConstraint(space: *const CPSpace, constraint: CPConstraint);
    pub fn cpSpaceRemoveConstraint(space: *const CPSpace, constraint: CPConstraint);
    pub fn cpSpaceContainsConstraint(space: *const CPSpace, constraint: CPConstraint) -> bool;

    pub fn cpSpaceReindexShape(space: *const CPSpace, shape: *const CPShape);
    pub fn cpSpaceReindexShapesForBody(space: *const CPSpace, body: *const CPBody);
    pub fn cpSpaceReindexStatic(space: *const CPSpace);

    // TODO iterators for body/shape/constraint

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
    // pub fn cpShapeGetBB(shape: *const CPShape) -> cpBB;
    // pub fn cpShapeGetSensor(shape: *const CPShape) -> cpBool;
    // pub fn cpShapeSetSensor(shape: *const CPShape, value: cpBool);
    // pub fn cpShapeGetSurfaceVelocity(shape: *const CPShape) -> CPVect;
    // pub fn cpShapeSetSurfaceVelocity(shape: *const CPShape, value: CPVect);
    // pub fn cpShapeGetCollisionType(shape: *const CPShape) -> cpCollisionType;
    // pub fn cpShapeSetCollisionType(shape: *const CPShape, value: cpCollisionType);
    // pub fn cpShapeGetFilter(shape: *const CPShape) -> cpShapeFilter;
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
    pub fn cpSegmentShapeNew(body: *const CPBody, a: CPVect, b: CPVect, radius: CPFloat) -> *const CPShape;
    pub fn cpSegmentShapeGetA(shape: *const CPShape) -> CPVect ;
    pub fn cpSegmentShapeGetB(shape: *const CPShape) -> CPVect ;
    pub fn cpSegmentShapeGetNormal(shape: *const CPShape) -> CPVect ;
    pub fn cpSegmentShapeGetRadius(shape: *const CPShape) -> CPFloat ;
    pub fn cpSegmentShapeSetNeighbors(shape: *const CPShape, prev: CPVect, next: CPVect);

    // TODO
    // Polygon Shape
    //pub fn cpPolyShapeNew(body: *const CPBody, numVerts: c_int, verts: *const CPVect , transform: cpTransform, radius: CPFloat) -> *const CPPolyShape;
    //pub fn cpPolyShapeGetNumVerts(shape: *const CPPolyShape) -> c_int;
    //pub fn cpPolyShapeGetVert(shape: *const CPPolyShape, index: c_int) -> CPVect;
    //pub fn cpPolyShapeGetRadius() -> CPFloat;
    //pub fn cpCentroidForPoly(count: c_int, verts: *const CPVect) -> CPVect;

    // Box Shape
    pub fn cpBoxShapeNew(body: *const CPBody, width: CPFloat, height: CPFloat, radius: CPFloat) -> *const CPPolyShape;
}
