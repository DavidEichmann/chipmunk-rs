
#![warn(missing_docs)]

//! # Bindings to the Chipmunk2D physics library
//!
//! Tested on Chipmunk2D 7.0.1
//!
//! These bindings provide a memory safety, but otherwise attempts to stay close to the
//! original API. see the official C API (manual)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/] for details on how to use the API.
//! These bindings are not zero-cost. Some overhead is due to memory safety related code. This cost may be reduces in future releases.
//!
//! # What's not done yet?
//!
//! * Only Circle and Box(Poly) Shapes are implemented at the moemnt.
//! * Iterators/callbacks.
//! * Cyclic references. How can cyclic references be providede while maintaining memory safety (e.g. `body.space()`, `shape.body()`, `shape.body()`).
//! * This! TODO: Have a thurough run through the (chipmunk documentation)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles] and note missing functionality.

extern crate libc;

pub mod ffi;

use std::rc::*;
use std::cell::*;
use std::marker::{PhantomData};
use std::collections::hash_map::*;

use ffi::*;

/// Implement this trait to define what types you intend to use with this library.
///
/// # Examples
///
/// To use a tuple (x, y) as the vector type simple do:
///
/// ```
/// use chipmunk_rs::{ ChipmunkRsTypes };
/// use chipmunk_rs::ffi::{ CPVect };
///
/// enum TupleTypes {}
/// impl ChipmunkRsTypes for TupleTypes {
/// type Vect = (f64, f64);
///     fn to_vect(cp_vect: CPVect) -> Self::Vect {
///         (cp_vect.x, cp_vect.y)
///     }
///
///     fn to_cp_vect(vect: &Self::Vect) -> CPVect {
///         CPVect::new(vect.0, vect.1)
///     }
/// }
/// ```
pub trait ChipmunkRsTypes {
    /// The 2D vector type.
    type Vect;

    /// Convert a `CPVect` to the `Vect` type.
    fn to_vect(cp_vect: CPVect) -> Self::Vect;

    /// Convert a `Vect` to the `CPVect` type.
    fn to_cp_vect(vect: &Self::Vect) -> CPVect;
}

/// Reference counted handle to a `Body<T>`.
type BodyHandle<T> = Rc<RefCell<Body<T>>>;

/// Reference counted handle to a `Shape<T>`.
type ShapeHandle<T> = Rc<RefCell<Box<Shape<T>>>>;

/// A 2D space. See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
pub struct Space<T: ChipmunkRsTypes> {
    ptr: *const CPSpace,
    bodies: HashMap<*const CPBody, BodyHandle<T>>,
    shapes: HashMap<*const CPShape, ShapeHandle<T>>
}

impl<T: ChipmunkRsTypes> Space<T> {
    /// Create a new sapce. You should set `T` explicitelly (e.g. `let mut space: Space<TupleTypes> = Space::new();`).
    pub fn new() -> Space<T> {
        Space {
            ptr: unsafe { cpSpaceNew() },
            bodies: HashMap::new(),
            shapes: HashMap::new()
        }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn gravity(&self) -> T::Vect {
        unsafe { T::to_vect(cpSpaceGetGravity(self.ptr)) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn is_locked(&self) -> bool {
        unsafe { cpSpaceIsLocked(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn current_time_step(&self) -> f64 {
        unsafe { cpSpaceGetCurrentTimeStep(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn iterations(&self) -> i32 {
        unsafe { cpSpaceGetIterations(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn damping(&self) -> f64 {
        unsafe { cpSpaceGetDamping(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn idle_speed_threshold(&self) -> f64 {
        unsafe { cpSpaceGetIdleSpeedThreshold(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn sleep_time_threshold(&self) -> f64 {
        unsafe { cpSpaceGetSleepTimeThreshold(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn collision_slop(&self) -> f64 {
        unsafe { cpSpaceGetCollisionSlop(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn collision_bias(&self) -> f64 {
        unsafe { cpSpaceGetCollisionBias(self.ptr) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn collision_persistence(&self) -> u32 {
        unsafe { cpSpaceGetCollisionPersistence(self.ptr) }
    }

    // TODO the result should not be deconstructed. the Space handles that automatically.
    // /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    // pub fn static_body(&self) -> &Body {
    //     unsafe { &Body { ptr: cpSpaceGetStaticBody(self.ptr) } }
    // }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_gravity(&self, g: T::Vect) {
        unsafe { cpSpaceSetGravity(self.ptr, T::to_cp_vect(&g)) };
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_iterations(&mut self, value: i32)
    {
        unsafe { cpSpaceSetIterations(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_damping(&mut self, value: f64)
    {
        unsafe { cpSpaceSetDamping(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_idle_speed_threshold(&mut self, value: f64)
    {
        unsafe { cpSpaceSetIdleSpeedThreshold(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_sleep_time_threshold(&mut self, value: f64)
    {
        unsafe { cpSpaceSetSleepTimeThreshold(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_collision_slop(&mut self, value: f64)
    {
        unsafe { cpSpaceSetCollisionSlop(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_collision_bias(&mut self, value: f64)
    {
        unsafe { cpSpaceSetCollisionBias(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn set_collision_persistence(&mut self, value: u32)
    {
        unsafe { cpSpaceSetCollisionPersistence(self.ptr, value); }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn step(&mut self, dt: f64) {
        unsafe { cpSpaceStep(self.ptr, dt) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace]. Make sure to
    /// also insert any attached body (this is not done for you).
    pub fn add_shape(&mut self, shape: Box<Shape<T>>) -> ShapeHandle<T> {
        // Add the shape to the space in C.
        unsafe { cpSpaceAddShape(self.ptr, shape.to_shape()); }

        // Add the shape to the space's managed shapes
        let handle = Rc::new(RefCell::new(shape));
        if let Some(_) = self.shapes.insert(handle.borrow().to_shape(), handle.clone()) {
            panic!("Trying to insert an already instered shape in to a space. This should not be possible as Shape::new() is the only way to create owned shapes and it always creates new shapes.");
        }

        handle
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn add_body(&mut self, body: Body<T>) -> BodyHandle<T> {
        // Add the body to the space in C.
        unsafe { cpSpaceAddBody(self.ptr, body.ptr); }

        // Add the body to the space's managed shapes
        let handle = Rc::new(RefCell::new(body));
        if let Some(_) = self.bodies.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Trying to insert an already instered body in to a space. This should not be possible as Body::new() is the only way to create owned bodies and it always creates new bodies.");
        }

        handle
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn remove_shape(&mut self, shape: ShapeHandle<T>) -> Option<ShapeHandle<T>> {
        // Remove the shape from the space in C.
        unsafe { cpSpaceRemoveShape(self.ptr, shape.borrow().to_shape()); }

        // Remove the shape from the space's managed shapes
        self.shapes.remove(&shape.borrow().to_shape())
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn remove_body(&mut self, body: BodyHandle<T>) -> Option<BodyHandle<T>> {
        // Remove the body from the space in C.
        unsafe { cpSpaceRemoveBody(self.ptr, body.borrow().ptr); }

        // Remove the body from the space's managed shapes
        self.bodies.remove(&body.borrow().ptr)
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn contains_shape(&self, shape: ShapeHandle<T>) -> bool {
        unsafe { cpSpaceContainsShape(self.ptr, shape.borrow().to_shape()) }
    }

    /// See (Chipmunk Spaces)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace].
    pub fn contains_body(&self, body: BodyHandle<T>) -> bool {
        unsafe { cpSpaceContainsBody(self.ptr, body.borrow().ptr) }
    }

    // TODO Constraints
    // cpConstraint *cpSpaceAddConstraint(&self, constraint: cpConstraint)
    // pub fn remove_constraint(&self, constraint: cpConstraint);
    // cpBool cpSpaceContainsConstraint(&self, constraint: cpConstraint)

    // TODO reindexing
    //pub fn reindex_shape<S: Shape<T>>(&mut self, shape: S) {
    //    unsafe { cpSpaceReindexShape(self.ptr, shape.to_shape()); }
    //}
    //pub fn reindex_shapes_for_body(&mut self, body: Body<T>) {
    //    unsafe { cpSpaceReindexShapesForBody(self.ptr, body.ptr); }
    //}
    //pub fn reindex_static(&mut self) {
    //    unsafe { cpSpaceReindexStatic(self.ptr, ); }
    //}

    // TODO iterators for body/shape/constraint
}

impl<T: ChipmunkRsTypes> Drop for Space<T> {
    fn drop(&mut self) {
        unsafe { cpSpaceFree(self.ptr); }

        // TODO free all resources (e.g. bodies, shapes, constraints, etc).
    }
}

/// A physics Body. See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
pub struct Body<T: ChipmunkRsTypes> {
    _marker: PhantomData<T>,
    ptr: *const CPBody
}

impl<T: ChipmunkRsTypes> Body<T> {
    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn new_dynamic(m: f64, i: f64) -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNew(m, i)
        } }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn new_static() -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNewStatic()
        } }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn new_kinematic() -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNewKinematic()
        } }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn position(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetPosition(self.ptr)) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn center_of_gravity(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetCenterOfGravity(self.ptr)) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn velocity(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetVelocity(self.ptr)) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn force(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetForce(self.ptr)) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn angle(&self) -> f64 {
        unsafe { cpBodyGetAngle(self.ptr) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn angular_velocity(&self) -> f64 {
        unsafe { cpBodyGetAngularVelocity(self.ptr) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn torque(&self) -> f64 {
        unsafe { cpBodyGetTorque(self.ptr) }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn rotation(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetRotation(self.ptr)) }
    }

    // TODO This is a circular reference. How can we maintain memory safety with this?
    // pub fn space(&self) -> Option<Space> {
    //     unsafe { cpBodyGetSpace(self.ptr).map(|ptr| Space { ptr: ptr }) }
    // }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_mass(&mut self, m: f64) {
        unsafe { cpBodySetMass(self.ptr, m); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_moment(&mut self, i: f64) {
        unsafe { cpBodySetMoment(self.ptr, i); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_position(&mut self, pos: T::Vect) {
        unsafe { cpBodySetPosition(self.ptr, T::to_cp_vect(&pos)); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_center_of_gravity(&mut self, cog: T::Vect) {
        unsafe { cpBodySetCenterOfGravity(self.ptr, T::to_cp_vect(&cog)); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_velocity(&mut self, value: T::Vect) {
        unsafe { cpBodySetVelocity(self.ptr, T::to_cp_vect(&value)); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_force(&mut self, value: T::Vect) {
        unsafe { cpBodySetForce(self.ptr, T::to_cp_vect(&value)); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_angle(&mut self, a: f64) {
        unsafe { cpBodySetAngle(self.ptr, a); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_angular_velocity(&mut self, value: f64) {
        unsafe { cpBodySetAngularVelocity(self.ptr, value); }
    }

    /// See (Chipmunk Rigid Bodies)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpBody].
    pub fn set_torque(&mut self, value: f64) {
        unsafe { cpBodySetTorque(self.ptr, value); }
    }
}

impl<T: ChipmunkRsTypes> Drop for Body<T> {
    fn drop(&mut self) {
        println!("Free!");
        unsafe { cpBodyFree(self.ptr); }
    }
}

// TODO is there a way to make ShapeBase private?
/// A collision Shape base trait. See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
/// All shape structs implement this trait.
pub trait BaseShape: Drop {
    /// Get the raw shape pointer.
    fn to_shape(&self) -> *const CPShape;
}

/// A collision Shape trait. See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
/// All shape structs implement this trait.
pub trait Shape<T: ChipmunkRsTypes>: BaseShape {

    /// Get the body attached to the shape.
    fn body(&self) -> BodyHandle<T>;

    // TODO Circular reference to space.
    // fn space(&self) -> &Space<T>;
    // fn space_mut(&mut self) -> &mut Space<T>;

    /// See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
    fn elasticity(&self) -> f64 {
        unsafe { cpShapeGetElasticity(self.to_shape()) }
    }

    /// See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
    fn friction(&self) -> f64 {
        unsafe { cpShapeGetFriction(self.to_shape()) }
    }

    /// See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
    fn set_body(&mut self, body: Body<T>) {
        unsafe { cpShapeSetBody(self.to_shape(), body.ptr); }
    }

    /// See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
    fn set_elasticity(&mut self, value: f64) {
        unsafe { cpShapeSetElasticity(self.to_shape(), value); }
    }

    /// See (Chipmunk Collision Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape].
    fn set_friction(&mut self, value: f64) {
        unsafe { cpShapeSetFriction(self.to_shape(), value); }
    }
}

/// A CircleShape. See (Working With Circle Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles]
pub struct CircleShape<T: ChipmunkRsTypes> {
    ptr: *const CPCircleShape,
    body: BodyHandle<T>
}

impl<T: ChipmunkRsTypes> CircleShape<T> {
    /// Create a new circle shape. See (Working With Circle Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles]
    pub fn new(body: BodyHandle<T>, radius: f64, offset: T::Vect) -> CircleShape<T> {
        unsafe {
            CircleShape {
                ptr: cpCircleShapeNew(body.borrow().ptr, radius, T::to_cp_vect(&offset)),
                body: body.clone()
            }
        }
    }

    /// See (Working With Circle Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles]
    pub fn offset(&self) -> T::Vect {
        unsafe { T::to_vect(cpCircleShapeGetOffset(self.ptr)) }
    }

    /// See (Working With Circle Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Circles]
    pub fn radius(&self) -> f64 {
        unsafe { cpCircleShapeGetRadius(self.ptr) }
    }
}

impl<T: ChipmunkRsTypes> Drop for CircleShape<T> {
    fn drop(&mut self) {
        unsafe { cpShapeFree(self.to_shape()); }
    }
}

impl<T: ChipmunkRsTypes> BaseShape for CircleShape<T> {
    fn to_shape(&self) -> *const CPShape {
        self.ptr as *const CPShape
    }
}

impl<T: ChipmunkRsTypes> Shape<T> for CircleShape<T> {
    fn body(&self) -> BodyHandle<T> {
        self.body.clone()
    }
}

/// A CircleShape. See (Working With Polygon Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys].
pub struct PolyShape<T: ChipmunkRsTypes> {
    ptr: *const CPPolyShape,
    body: BodyHandle<T>
}

impl<T: ChipmunkRsTypes> PolyShape<T> {
    /// Create a new box shape. radius is the radius of the corners. Use `radius: 0.0` for no rounded corners.
    /// See (Working With Polygon Shapes)[http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpShape-Polys].
    pub fn new_box(body: BodyHandle<T>, width: f64, height: f64, radius: f64) -> PolyShape<T> {
        unsafe {
            PolyShape {
                ptr: cpBoxShapeNew(body.borrow().ptr, width, height, radius),
                body: body.clone()
            }
        }
    }
}

impl<T: ChipmunkRsTypes> Drop for PolyShape<T> {
    fn drop(&mut self) {
        unsafe { cpShapeFree(self.to_shape()); }
    }
}

impl<T: ChipmunkRsTypes> BaseShape for PolyShape<T> {
    fn to_shape(&self) -> *const CPShape {
        self.ptr as *const CPShape
    }
}

impl<T: ChipmunkRsTypes> Shape<T> for PolyShape<T> {
    fn body(&self) -> BodyHandle<T> {
        self.body.clone()
    }
}




#[cfg(test)]
mod tests {

    use super::{ChipmunkRsTypes, CircleShape,Space,Body};
    use super::ffi::*;

    enum TupleTypes {}
    impl ChipmunkRsTypes for TupleTypes {
        type Vect = (f64, f64);
        fn to_vect(cp_vect: CPVect) -> Self::Vect {
            (cp_vect.x, cp_vect.y)
        }

        fn to_cp_vect(vect: &Self::Vect) -> CPVect {
            CPVect::new(vect.0, vect.1)
        }
    }

    #[test]
    fn gravity() {
        // Create a space with gravity.
        let mut space: Space<TupleTypes> = Space::new();
        space.set_gravity((1.0, 2.0));

        // Create a dynamic body and add it to the space.
        let body_handle = space.add_body(Body::new_dynamic(1.0, 1.0));
        space.add_shape(Box::new(CircleShape::new(body_handle.clone(), 1.0, (0.0, 0.0))));
        body_handle.borrow_mut().set_position((-1.0, -2.0));

        // Let the ball fall and check that the velocity/position change.
        assert_eq!((0.0, 0.0), body_handle.borrow().velocity());
        assert_eq!((-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((1.0, 2.0), body_handle.borrow().velocity());
        assert_eq!((-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((2.0, 4.0), body_handle.borrow().velocity());
        assert_eq!((0.0, 0.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((3.0, 6.0), body_handle.borrow().velocity());
        assert_eq!((2.0, 4.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((4.0, 8.0), body_handle.borrow().velocity());
        assert_eq!((5.0, 10.0), body_handle.borrow().position());

        // Reset velocity/position and check again
        body_handle.borrow_mut().set_position((-1.0, -2.0));
        body_handle.borrow_mut().set_velocity((0.0, 0.0));

        assert_eq!((0.0, 0.0), body_handle.borrow().velocity());
        assert_eq!((-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((1.0, 2.0), body_handle.borrow().velocity());
        assert_eq!((-1.0, -2.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((2.0, 4.0), body_handle.borrow().velocity());
        assert_eq!((0.0, 0.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((3.0, 6.0), body_handle.borrow().velocity());
        assert_eq!((2.0, 4.0), body_handle.borrow().position());
        space.step(1.0);
        assert_eq!((4.0, 8.0), body_handle.borrow().velocity());
        assert_eq!((5.0, 10.0), body_handle.borrow().position());
    }
}
