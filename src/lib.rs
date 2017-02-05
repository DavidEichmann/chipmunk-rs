
#![warn(missing_docs)]

//! # Bindings to the Chipmunk2D physics library
//!
//! Tested on Chipmunk2D 7.0.1
//!
//! These bindings provide a memory safety, but otherwise attempts to stay close to the
//! original API. See the official [Chipmunk2D 7.0.1 manual](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/) for details on how to use the API.
//! These bindings are not zero-cost. Some overhead is due to memory safety related code. This cost may be reduced in future releases. Alternativelly, the ffi module
//! provides raw API bindings (use at your own risk).
//!
//! # What's not done yet?
//!
//! * Shapes other than the Circle and Box (Poly) Shapes.
//! * Iterators/callbacks.
//! * Functions to get associated bodies/shapes/spaces from eachother. A clean and safe way of doing this in rust is needed.
//! * This! Have a thurough run through the [Chipmunk2D 7.0.1 manual](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/) and note missing functionality.

extern crate libc;

pub mod ffi;

use std::rc::*;
use std::cell::*;
use std::collections::hash_map::*;

use ffi::*;

/// Reference counted handle to a `Body`.
pub type BodyHandle = Rc<RefCell<Body>>;

/// Reference counted handle to a `Shape`.
pub type ShapeHandle = Rc<RefCell<Box<Shape>>>;

/// A 2D space. See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
pub struct Space {
    ptr: *const CPSpace,
    bodies: HashMap<*const CPBody, BodyHandle>,
    shapes: HashMap<*const CPShape, ShapeHandle>,
}

impl Space {
    /// Create a new sapce.
    pub fn new() -> Space {
        Space {
            ptr: unsafe { cpSpaceNew() },
            bodies: HashMap::new(),
            shapes: HashMap::new(),
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn gravity(&self) -> CPVect {
        unsafe { cpSpaceGetGravity(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn is_locked(&self) -> bool {
        unsafe { cpSpaceIsLocked(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn current_time_step(&self) -> f64 {
        unsafe { cpSpaceGetCurrentTimeStep(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn iterations(&self) -> i32 {
        unsafe { cpSpaceGetIterations(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn damping(&self) -> f64 {
        unsafe { cpSpaceGetDamping(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn idle_speed_threshold(&self) -> f64 {
        unsafe { cpSpaceGetIdleSpeedThreshold(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn sleep_time_threshold(&self) -> f64 {
        unsafe { cpSpaceGetSleepTimeThreshold(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn collision_slop(&self) -> f64 {
        unsafe { cpSpaceGetCollisionSlop(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn collision_bias(&self) -> f64 {
        unsafe { cpSpaceGetCollisionBias(self.ptr) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn collision_persistence(&self) -> u32 {
        unsafe { cpSpaceGetCollisionPersistence(self.ptr) }
    }

    // TODO the result should not be deconstructed. the Space handles that automatically.
    // /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    // pub fn static_body(&self) -> &Body {
    //     unsafe { &Body { ptr: cpSpaceGetStaticBody(self.ptr) } }
    // }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_gravity(&self, g: CPVect) {
        unsafe { cpSpaceSetGravity(self.ptr, g) };
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_iterations(&mut self, value: i32) {
        unsafe {
            cpSpaceSetIterations(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_damping(&mut self, value: f64) {
        unsafe {
            cpSpaceSetDamping(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_idle_speed_threshold(&mut self, value: f64) {
        unsafe {
            cpSpaceSetIdleSpeedThreshold(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_sleep_time_threshold(&mut self, value: f64) {
        unsafe {
            cpSpaceSetSleepTimeThreshold(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_collision_slop(&mut self, value: f64) {
        unsafe {
            cpSpaceSetCollisionSlop(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_collision_bias(&mut self, value: f64) {
        unsafe {
            cpSpaceSetCollisionBias(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn set_collision_persistence(&mut self, value: u32) {
        unsafe {
            cpSpaceSetCollisionPersistence(self.ptr, value);
        }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn step(&mut self, dt: f64) {
        unsafe { cpSpaceStep(self.ptr, dt) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace). Make sure to
    /// also insert any attached body (this is not done for you).
    pub fn add_shape(&mut self, shape: Box<Shape>) -> ShapeHandle {
        // Add the shape to the space in C.
        unsafe {
            cpSpaceAddShape(self.ptr, shape.to_shape());
        }

        // Add the shape to the space's managed shapes
        let handle = Rc::new(RefCell::new(shape));
        if let Some(_) = self.shapes.insert(handle.borrow().to_shape(), handle.clone()) {
            panic!("Trying to insert an already instered shape in to a space. This should not be \
                    possible as Shape::new() is the only way to create owned shapes and it \
                    always creates new shapes.");
        }

        handle
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn add_body(&mut self, body: Body) -> BodyHandle {
        // Add the body to the space in C.
        unsafe {
            cpSpaceAddBody(self.ptr, body.ptr);
        }

        // Add the body to the space's managed shapes
        let handle = Rc::new(RefCell::new(body));
        if let Some(_) = self.bodies.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Trying to insert an already instered body in to a space. This should not be \
                    possible as Body::new() is the only way to create owned bodies and it always \
                    creates new bodies.");
        }

        handle
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn remove_shape(&mut self, shape: ShapeHandle) -> Option<ShapeHandle> {
        // Remove the shape from the space in C.
        unsafe {
            cpSpaceRemoveShape(self.ptr, shape.borrow().to_shape());
        }

        // Remove the shape from the space's managed shapes
        self.shapes.remove(&shape.borrow().to_shape())
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn remove_body(&mut self, body: BodyHandle) -> Option<BodyHandle> {
        // Remove the body from the space in C.
        unsafe {
            cpSpaceRemoveBody(self.ptr, body.borrow().ptr);
        }

        // Remove the body from the space's managed shapes
        self.bodies.remove(&body.borrow().ptr)
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn contains_shape(&self, shape: ShapeHandle) -> bool {
        unsafe { cpSpaceContainsShape(self.ptr, shape.borrow().to_shape()) }
    }

    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn contains_body(&self, body: BodyHandle) -> bool {
        unsafe { cpSpaceContainsBody(self.ptr, body.borrow().ptr) }
    }

    // TODO Constraints
    // cpConstraint *cpSpaceAddConstraint(&self, constraint: cpConstraint)
    // pub fn remove_constraint(&self, constraint: cpConstraint);
    // cpBool cpSpaceContainsConstraint(&self, constraint: cpConstraint)

    // TODO reindexing
    //pub fn reindex_shape<S: Shape>(&mut self, shape: S) {
    //    unsafe { cpSpaceReindexShape(self.ptr, shape.to_shape()); }
    //}
    //pub fn reindex_shapes_for_body(&mut self, body: Body) {
    //    unsafe { cpSpaceReindexShapesForBody(self.ptr, body.ptr); }
    //}
    //pub fn reindex_static(&mut self) {
    //    unsafe { cpSpaceReindexStatic(self.ptr, ); }
    //}

    // TODO iterators for body/shape/constraint
}

impl Drop for Space {
    fn drop(&mut self) {
        unsafe {
            cpSpaceFree(self.ptr);
        }

        // TODO free all resources (e.g. bodies, shapes, constraints, etc).
    }
}

/// A physics Body. See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
pub struct Body {
    ptr: *const CPBody,
}

impl Body {
    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn new_dynamic(m: f64, i: f64) -> Body {
        unsafe { Body { ptr: cpBodyNew(m, i) } }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn new_static() -> Body {
        unsafe { Body { ptr: cpBodyNewStatic() } }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn new_kinematic() -> Body {
        unsafe { Body { ptr: cpBodyNewKinematic() } }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn position(&self) -> CPVect {
        unsafe { cpBodyGetPosition(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn center_of_gravity(&self) -> CPVect {
        unsafe { cpBodyGetCenterOfGravity(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn velocity(&self) -> CPVect {
        unsafe { cpBodyGetVelocity(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn force(&self) -> CPVect {
        unsafe { cpBodyGetForce(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn angle(&self) -> f64 {
        unsafe { cpBodyGetAngle(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn angular_velocity(&self) -> f64 {
        unsafe { cpBodyGetAngularVelocity(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn torque(&self) -> f64 {
        unsafe { cpBodyGetTorque(self.ptr) }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn rotation(&self) -> CPVect {
        unsafe { cpBodyGetRotation(self.ptr) }
    }

    // TODO This is a circular reference. How can we maintain memory safety with this?
    // pub fn space(&self) -> Option<Space> {
    //     unsafe { cpBodyGetSpace(self.ptr).map(|ptr| Space { ptr: ptr }) }
    // }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_mass(&mut self, m: f64) {
        unsafe {
            cpBodySetMass(self.ptr, m);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_moment(&mut self, i: f64) {
        unsafe {
            cpBodySetMoment(self.ptr, i);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_position(&mut self, pos: CPVect) {
        unsafe {
            cpBodySetPosition(self.ptr, pos);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_center_of_gravity(&mut self, cog: CPVect) {
        unsafe {
            cpBodySetCenterOfGravity(self.ptr, cog);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_velocity(&mut self, value: CPVect) {
        unsafe {
            cpBodySetVelocity(self.ptr, value);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_force(&mut self, value: CPVect) {
        unsafe {
            cpBodySetForce(self.ptr, value);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_angle(&mut self, a: f64) {
        unsafe {
            cpBodySetAngle(self.ptr, a);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_angular_velocity(&mut self, value: f64) {
        unsafe {
            cpBodySetAngularVelocity(self.ptr, value);
        }
    }

    /// See [Chipmunk Rigid Bodies](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody).
    pub fn set_torque(&mut self, value: f64) {
        unsafe {
            cpBodySetTorque(self.ptr, value);
        }
    }
}

impl Drop for Body {
    fn drop(&mut self) {
        unsafe {
            cpBodyFree(self.ptr);
        }
    }
}

// TODO is there a way to make ShapeBase private?
/// A collision Shape base trait. See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
/// All shape structs implement this trait.
pub trait BaseShape: Drop {
    /// Get the raw shape pointer.
    fn to_shape(&self) -> *const CPShape;
}

/// A collision Shape trait. See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
/// All shape structs implement this trait.
pub trait Shape: BaseShape {
    /// Get the body attached to the shape.
    fn body(&self) -> BodyHandle;

    // TODO Circular reference to space.
    // fn space(&self) -> &Space;
    // fn space_mut(&mut self) -> &mut Space;

    /// See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
    fn elasticity(&self) -> f64 {
        unsafe { cpShapeGetElasticity(self.to_shape()) }
    }

    /// See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
    fn friction(&self) -> f64 {
        unsafe { cpShapeGetFriction(self.to_shape()) }
    }

    /// See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
    fn set_body(&mut self, body: Body) {
        unsafe {
            cpShapeSetBody(self.to_shape(), body.ptr);
        }
    }

    /// See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
    fn set_elasticity(&mut self, value: f64) {
        unsafe {
            cpShapeSetElasticity(self.to_shape(), value);
        }
    }

    /// See [Chipmunk Collision Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape).
    fn set_friction(&mut self, value: f64) {
        unsafe {
            cpShapeSetFriction(self.to_shape(), value);
        }
    }
}

/// A CircleShape. See [Working With Circle Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles)
pub struct CircleShape {
    ptr: *const CPCircleShape,
    body: BodyHandle,
}

impl CircleShape {
    /// Create a new circle shape. See [Working With Circle Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles)
    pub fn new(body: BodyHandle, radius: f64, offset: CPVect) -> CircleShape {
        unsafe {
            CircleShape {
                ptr: cpCircleShapeNew(body.borrow().ptr, radius, offset),
                body: body.clone(),
            }
        }
    }

    /// See [Working With Circle Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles)
    pub fn offset(&self) -> CPVect {
        unsafe { cpCircleShapeGetOffset(self.ptr) }
    }

    /// See [Working With Circle Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles)
    pub fn radius(&self) -> f64 {
        unsafe { cpCircleShapeGetRadius(self.ptr) }
    }
}

impl Drop for CircleShape {
    fn drop(&mut self) {
        unsafe {
            cpShapeFree(self.to_shape());
        }
    }
}

impl BaseShape for CircleShape {
    fn to_shape(&self) -> *const CPShape {
        self.ptr as *const CPShape
    }
}

impl Shape for CircleShape {
    fn body(&self) -> BodyHandle {
        self.body.clone()
    }
}

/// A PolyShape. See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
pub struct PolyShape {
    ptr: *const CPPolyShape,
    body: BodyHandle,
}

impl PolyShape {
    /// Create a new poly shape. radius is the radius of the corners. Use `radius: 0.0` for no rounded corners.
    /// See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
    pub fn new_convex_raw(body: BodyHandle, points: &[CPVect], radius: f64) -> PolyShape {
        unsafe {
            PolyShape {
                ptr: cpPolyShapeNewRaw(body.borrow().ptr,
                                       points.len() as i32,
                                       points.as_ptr(),
                                       radius),
                body: body.clone(),
            }
        }
    }

    /// Create a new box shape. radius is the radius of the corners. Use `radius: 0.0` for no rounded corners.
    /// See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
    pub fn new_box(body: BodyHandle, width: f64, height: f64, radius: f64) -> PolyShape {
        unsafe {
            PolyShape {
                ptr: cpBoxShapeNew(body.borrow().ptr, width, height, radius),
                body: body.clone(),
            }
        }
    }

    /// See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
    pub fn len(&self) -> usize {
        unsafe { cpPolyShapeGetCount(self.ptr) as usize }
    }

    /// See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
    pub fn get(&self, index: usize) -> Option<CPVect> {
        if index < self.len() {
            Some(unsafe { cpPolyShapeGetVert(self.ptr, index as i32) })
        } else {
            None
        }
    }

    /// See [Working With Polygon Shapes](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys).
    pub fn radius(&self) -> f64 {
        unsafe { cpPolyShapeGetRadius(self.ptr) }
    }
}

impl Drop for PolyShape {
    fn drop(&mut self) {
        unsafe {
            cpShapeFree(self.to_shape());
        }
    }
}

impl BaseShape for PolyShape {
    fn to_shape(&self) -> *const CPShape {
        self.ptr as *const CPShape
    }
}

impl Shape for PolyShape {
    fn body(&self) -> BodyHandle {
        self.body.clone()
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn poly_shape() {
        let mut space: Space = Space::new();
        let body = space.add_body(Body::new_dynamic(1.0, 1.0));
        let points = [CPVect::new(1.0, 1.0), CPVect::new(2.0, 1.0), CPVect::new(1.5, 2.0)];
        let poly = {
            let points_clone = points.clone();
            PolyShape::new_convex_raw(body, &points_clone, 0.1)
        };

        assert_eq!(0.1, poly.radius());
        assert_eq!(points.len(), poly.len());
        assert_eq!(None, poly.get(points.len()));
        assert_eq!(None, poly.get(points.len() + 1));
        assert_eq!(None, poly.get(points.len() + 2));
        for (i, &p) in points.iter().enumerate() {
            assert_eq!(Some(p), poly.get(i));
        }
    }

    #[test]
    fn gravity() {
        // Create a space with gravity.
        let mut space: Space = Space::new();
        space.set_gravity(CPVect::new(1.0, 2.0));

        // Create a dynamic body and add it to the space.
        let body_handle = space.add_body(Body::new_dynamic(1.0, 1.0));
        space.add_shape(Box::new(CircleShape::new(body_handle.clone(), 1.0, CPVect::new(0.0, 0.0))));
        body_handle.borrow_mut().set_position(CPVect::new(-1.0, -2.0));

        // Let the ball fall and check that the velocity/position change.
        assert_eq!(CPVect::new(0.0, 0.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(1.0, 2.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(2.0, 4.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(0.0, 0.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(3.0, 6.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(2.0, 4.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(4.0, 8.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(5.0, 10.0), body_handle.borrow().position());

        // Reset velocity/position and check again
        body_handle.borrow_mut().set_position(CPVect::new(-1.0, -2.0));
        body_handle.borrow_mut().set_velocity(CPVect::new(0.0, 0.0));

        assert_eq!(CPVect::new(0.0, 0.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(1.0, 2.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(2.0, 4.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(0.0, 0.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(3.0, 6.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(2.0, 4.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!(CPVect::new(4.0, 8.0), body_handle.borrow().velocity());
        assert_eq!(CPVect::new(5.0, 10.0), body_handle.borrow().position());
    }
}
