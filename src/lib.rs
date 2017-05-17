
#![warn(missing_docs)]

//! # Bindings to the Chipmunk2D physics library
//!
//! Tested on Chipmunk2D 7.0.1
//!
//! These bindings add memory safety but otherwise attempts to stay close to the
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

/// A 2D space. See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
pub struct Space {
    ptr: *const CPSpace,
    bodies: HashMap<*const CPBody, BodyHandle>,
    shapes: HashMap<*const CPShape, ShapeHandle>,
    pin_joints: HashMap<*const CPPinJoint, PinJointHandle>,
    slide_joints: HashMap<*const CPSlideJoint, SlideJointHandle>,
    pivot_joints: HashMap<*const CPPivotJoint, PivotJointHandle>,
    groove_joints: HashMap<*const CPGrooveJoint, GrooveJointHandle>,
    damped_springs: HashMap<*const CPDampedSpring, DampedSpringHandle>,
    damped_rotary_springs: HashMap<*const CPDampedRotarySpring, DampedRotarySpringHandle>,
    rotary_limit_joints: HashMap<*const CPRotaryLimitJoint, RotaryLimitJointHandle>,
    ratchet_joints: HashMap<*const CPRatchetJoint, RatchetJointHandle>,
    gear_joints: HashMap<*const CPGearJoint, GearJointHandle>,
    simple_motors: HashMap<*const CPSimpleMotor, SimpleMotorHandle>,
}

impl Space {
    /// Create a new sapce.
    pub fn new() -> Space {
        Space {
            ptr: unsafe { cpSpaceNew() },
            bodies: HashMap::new(),
            shapes: HashMap::new(),
            pin_joints: HashMap::new(),
            slide_joints: HashMap::new(),
            pivot_joints: HashMap::new(),
            groove_joints: HashMap::new(),
            damped_springs: HashMap::new(),
            damped_rotary_springs: HashMap::new(),
            rotary_limit_joints: HashMap::new(),
            ratchet_joints: HashMap::new(),
            gear_joints: HashMap::new(),
            simple_motors: HashMap::new(),
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

    /// Add a PinJoint constraint.
    pub fn add_pin_joint(&mut self, pin_joint: PinJoint) -> PinJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, pin_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(pin_joint));
        if let Some(_) = self.pin_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted pin_joint.")
        }

        handle
    }

    /// Add a SlideJoint constraint.
    pub fn add_slide_joint(&mut self, slide_joint: SlideJoint) -> SlideJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, slide_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(slide_joint));
        if let Some(_) = self.slide_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted slide_joint.")
        }

        handle
    }

    /// Add a PivotJoint constraint.
    pub fn add_pivot_joint(&mut self, pivot_joint: PivotJoint) -> PivotJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, pivot_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(pivot_joint));
        if let Some(_) = self.pivot_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted pivot_joint.")
        }

        handle
    }

    /// Add a GrooveJoint constraint.
    pub fn add_groove_joint(&mut self, groove_joint: GrooveJoint) -> GrooveJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, groove_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(groove_joint));
        if let Some(_) = self.groove_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted groove_joint.")
        }

        handle
    }

    /// Add a DampedSpring constraint.
    pub fn add_damped_spring(&mut self, damped_spring: DampedSpring) -> DampedSpringHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, damped_spring.to_constraint()) }
        let handle = Rc::new(RefCell::new(damped_spring));
        if let Some(_) = self.damped_springs.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted , .")
        }

        handle
    }

    /// Add a DampedRotarySpring constraint.
    pub fn add_damped_rotary_spring(&mut self, damped_rotary_spring: DampedRotarySpring) -> DampedRotarySpringHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, damped_rotary_spring.to_constraint()) }
        let handle = Rc::new(RefCell::new(damped_rotary_spring));
        if let Some(_) = self.damped_rotary_springs.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted , .")
        }

        handle
    }

    /// Add a RotaryLimitJoint constraint.
    pub fn add_rotary_limit_joint(&mut self, rotary_limit_joint: RotaryLimitJoint) -> RotaryLimitJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, rotary_limit_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(rotary_limit_joint));
        if let Some(_) = self.rotary_limit_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted , .")
        }

        handle
    }

    /// Add a RatchetJoint constraint.
    pub fn add_ratchet_joint(&mut self, ratchet_joint: RatchetJoint) -> RatchetJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, ratchet_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(ratchet_joint));
        if let Some(_) = self.ratchet_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted , .")
        }

        handle
    }

    /// Add a GearJoint constraint.
    pub fn add_gear_joint(&mut self, gear_joint: GearJoint) -> GearJointHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, gear_joint.to_constraint()) }
        let handle = Rc::new(RefCell::new(gear_joint));
        if let Some(_) = self.gear_joints.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted gear_joint.")
        }

        handle
    }

    /// Add a SimpleMotor constraint.
    pub fn add_simple_motor(&mut self, simple_motor: SimpleMotor) -> SimpleMotorHandle {
        unsafe { cpSpaceAddConstraint(self.ptr, simple_motor.to_constraint()) }
        let handle = Rc::new(RefCell::new(simple_motor));
        if let Some(_) = self.simple_motors.insert(handle.borrow().ptr, handle.clone()) {
            panic!("Inserting an already inserted simple_motor.")
        }

        handle
    }

    // cpConstraint *cpSpaceAddConstraint(&self, constraint: cpConstraint)
    // pub fn remove_constraint(&self, constraint: cpConstraint);
    
    /// See [Chipmunk Spaces](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpSpace).
    pub fn contains_constraint<C>(&self, constraint: C) -> bool
        where C: Constraint
    {
        unsafe { cpSpaceContainsConstraint(self.ptr, constraint.to_constraint()) }
    }

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
    }
}

/// Reference counted handle to a `Body`.
pub type BodyHandle = Rc<RefCell<Body>>;

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

/// Trait for all constraint types.
pub trait Constraint: Drop {
    /// Convert this constraint to a constraint pointer.
    fn to_constraint(&self) -> *const CPConstraint;

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn max_force(&self) -> CPFloat {
        unsafe { cpConstraintGetMaxForce(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn error_bias(&self) -> CPFloat {
        unsafe { cpConstraintGetErrorBias(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn max_bias(&self) -> CPFloat {
        unsafe { cpConstraintGetMaxBias(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn collide_bodies(&self) -> bool {
        unsafe { cpConstraintGetCollideBodies(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn set_max_force(&self, value: CPFloat) {
        unsafe { cpConstraintSetMaxForce(self.to_constraint(), value) }
    }
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn set_error_bias(&self, value: CPFloat) {
        unsafe { cpConstraintSetErrorBias(self.to_constraint(), value) }
    }
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn set_max_bias(&self, value: CPFloat) {
        unsafe { cpConstraintSetMaxBias(self.to_constraint(), value) }
    }
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#cpConstraint-Properties).
    fn set_collide_bodies(&self, collide_bodies: bool) {
        unsafe { cpConstraintSetCollideBodies(self.to_constraint(), collide_bodies) }
    }
}

/// Reference counted handle to a `XXX`.
pub type PinJointHandle = Rc<RefCell<PinJoint>>;

/// A pin joint.
pub struct PinJoint {
    ptr: *const CPPinJoint,
}

impl PinJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor_a: CPVect, anchor_b: CPVect) -> PinJoint {
        unsafe {
            PinJoint {
                ptr: cpPinJointNew(body_a.borrow().ptr, body_b.borrow().ptr, anchor_a, anchor_b) as *const CPPinJoint

            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn anchor_a(&self) -> CPVect {
        unsafe { cpPinJointGetanchorA(self.to_constraint()) }
    }
    
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn anchor_b(&self) -> CPVect {
        unsafe { cpPinJointGetanchorB(self.to_constraint()) }
    }
    
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn dist(&self) -> CPFloat {
        unsafe { cpPinJointGetDist(self.to_constraint()) }
    }
    
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn setanchor_a(&self, value: CPVect) {
        unsafe { cpPinJointSetanchorA(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn setanchor_b(&self, value: CPVect) {
        unsafe { cpPinJointSetanchorB(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPinJoint).
    pub fn set_dist(&self, value: CPFloat) {
        unsafe { cpPinJointSetDist(self.to_constraint(), value) }
    }
}

impl Constraint for PinJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for PinJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `SlideJoint`.
pub type SlideJointHandle = Rc<RefCell<SlideJoint>>;

/// A Slide Joint.
pub struct SlideJoint {
    ptr: *const CPSlideJoint,
}

impl SlideJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor_a: CPVect, anchor_b: CPVect, min: CPFloat, max: CPFloat) -> SlideJoint {
        unsafe {
            SlideJoint {
                ptr: cpSlideJointNew(body_a.borrow().ptr, body_b.borrow().ptr, anchor_a, anchor_b, min, max) as *const CPSlideJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn anchor_a(&self) -> CPVect {
        unsafe { cpSlideJointGetanchorA(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn anchor_b(&self) -> CPVect {
        unsafe { cpSlideJointGetanchorB(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn min(&self) -> CPFloat {
        unsafe { cpSlideJointGetMin(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn max(&self) -> CPFloat {
        unsafe { cpSlideJointGetMax(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn set_anchor_a(&self, value: CPVect) {
        unsafe { cpSlideJointSetanchorA(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn set_anchor_b(&self, value: CPVect) {
        unsafe { cpSlideJointSetanchorB(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn set_min(&self, value: CPFloat) {
        unsafe { cpSlideJointSetMin(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSlideJoint).
    pub fn set_max(&self, value: CPFloat) {
        unsafe { cpSlideJointSetMax(self.to_constraint(), value) }
    }
}

impl Constraint for SlideJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for SlideJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `PivotJoint`.
pub type PivotJointHandle = Rc<RefCell<PivotJoint>>;

/// A Pivot Joint.
pub struct PivotJoint {
    ptr: *const CPPivotJoint,
}

impl PivotJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPivotJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor_a: CPVect, anchor_b: CPVect) -> PivotJoint {
        unsafe {
            PivotJoint {
                ptr: cpPivotJointNew2(body_a.borrow().ptr, body_b.borrow().ptr, anchor_a, anchor_b) as *const CPPivotJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPivotJoint).
    pub fn anchor_a(&self) -> CPVect {
        unsafe { cpPivotJointGetanchorA(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPivotJoint).
    pub fn anchor_b(&self) -> CPVect {
        unsafe { cpPivotJointGetanchorB(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPivotJoint).
    pub fn set_anchor_a(&self, value: CPVect) {
        unsafe { cpPivotJointSetanchorA(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpPivotJoint).
    pub fn set_anchor_b(&self, value: CPVect) {
        unsafe { cpPivotJointSetanchorB(self.to_constraint(), value) }
    }
}

impl Constraint for PivotJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for PivotJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `GrooveJoint`.
pub type GrooveJointHandle = Rc<RefCell<GrooveJoint>>;

/// A Groove Joint.
pub struct GrooveJoint {
    ptr: *const CPGrooveJoint,
}

impl GrooveJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, groove_a: CPVect, groove_b: CPVect, anchor_b: CPVect) -> GrooveJoint {
        unsafe {
            GrooveJoint {
                ptr: cpGrooveJointNew(body_a.borrow().ptr, body_b.borrow().ptr, groove_a, groove_b, anchor_b) as *const CPGrooveJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn groove_a(&self) -> CPVect {
        unsafe { cpGrooveJointGetGrooveA(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn groove_b(&self) -> CPVect {
        unsafe { cpGrooveJointGetGrooveB(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn anchor_b(&self) -> CPVect {
        unsafe { cpGrooveJointGetanchorB(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn set_groove_a(&self, value: CPVect) {
        unsafe { cpGrooveJointSetGrooveA(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn set_groove_b(&self, value: CPVect) {
        unsafe { cpGrooveJointSetGrooveB(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGrooveJoint).
    pub fn set_anchor_b(&self, value: CPVect) {
        unsafe { cpGrooveJointSetanchorB(self.to_constraint(), value) }
    }
}

impl Constraint for GrooveJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for GrooveJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `DampedSpring`.
pub type DampedSpringHandle = Rc<RefCell<DampedSpring>>;

/// A Damped Spring.
pub struct DampedSpring {
    ptr: *const CPDampedSpring,
}

impl DampedSpring {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor_a: CPVect, anchor_b: CPVect, rest_length: CPFloat, stiffness: CPFloat, damping: CPFloat) -> DampedSpring {
        unsafe {
            DampedSpring {
                ptr: cpDampedSpringNew(body_a.borrow().ptr, body_b.borrow().ptr, anchor_a, anchor_b, rest_length, stiffness, damping) as *const CPDampedSpring
            }
        }
    }
    
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn anchor_a(&self) -> CPVect {
        unsafe { cpDampedSpringGetanchorA(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn anchor_b(&self) -> CPVect {
        unsafe { cpDampedSpringGetanchorB(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn rest_length(&self) -> CPFloat {
        unsafe { cpDampedSpringGetRestLength(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn stiffness(&self) -> CPFloat {
        unsafe { cpDampedSpringGetStiffness(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn damping(&self) -> CPFloat {
        unsafe { cpDampedSpringGetDamping(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn set_anchor_a(&self, value: CPVect) {
        unsafe { cpDampedSpringSetanchorA(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn set_anchor_b(&self, value: CPVect) {
        unsafe { cpDampedSpringSetanchorB(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn set_rest_length(&self, value: CPFloat) {
        unsafe { cpDampedSpringSetRestLength(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn set_stiffness(&self, value: CPFloat) {
        unsafe { cpDampedSpringSetStiffness(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedSpring).
    pub fn set_damping(&self, value: CPFloat) {
        unsafe { cpDampedSpringSetDamping(self.to_constraint(), value) }
    }
}

impl Constraint for DampedSpring {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for DampedSpring {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `DampedRotarySpring`.
pub type DampedRotarySpringHandle = Rc<RefCell<DampedRotarySpring>>;

/// A Damped Rotary Spring.
pub struct DampedRotarySpring {
    ptr: *const CPDampedRotarySpring,
}

impl DampedRotarySpring {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, rest_angle: CPFloat, stiffness: CPFloat, damping: CPFloat) -> DampedRotarySpring {
        unsafe {
            DampedRotarySpring {
                ptr: cpDampedRotarySpringNew(body_a.borrow().ptr, body_b.borrow().ptr, rest_angle, stiffness, damping) as *const CPDampedRotarySpring
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn rest_angle(&self) -> CPFloat {
        unsafe { cpDampedRotarySpringGetRestAngle(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn stiffness(&self) -> CPFloat {
        unsafe { cpDampedRotarySpringGetStiffness(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn damping(&self) -> CPFloat {
        unsafe { cpDampedRotarySpringGetDamping(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn set_rest_angle(&self, value: CPFloat) {
        unsafe { cpDampedRotarySpringSetRestAngle(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn set_stiffness(&self, value: CPFloat) {
        unsafe { cpDampedRotarySpringSetStiffness(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpDampedRotarySpring).
    pub fn set_damping(&self, value: CPFloat) {
        unsafe { cpDampedRotarySpringSetDamping(self.to_constraint(), value) }
    }
}

impl Constraint for DampedRotarySpring {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for DampedRotarySpring {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `RotaryLimitJoint`.
pub type RotaryLimitJointHandle = Rc<RefCell<RotaryLimitJoint>>;

/// A Rotary Limit Joint.
pub struct RotaryLimitJoint {
    ptr: *const CPRotaryLimitJoint,
}

impl RotaryLimitJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRotaryLimitJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, min: CPFloat, max: CPFloat) -> RotaryLimitJoint {
        unsafe {
            RotaryLimitJoint {
                ptr: cpRotaryLimitJointNew(body_a.borrow().ptr, body_b.borrow().ptr, min, max) as *const CPRotaryLimitJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRotaryLimitJoint).
    pub fn min(&self) -> CPFloat {
        unsafe { cpRotaryLimitJointGetMin(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRotaryLimitJoint).
    pub fn max(&self) -> CPFloat {
        unsafe { cpRotaryLimitJointGetMax(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRotaryLimitJoint).
    pub fn set_min(&self, value: CPFloat) {
        unsafe { cpRotaryLimitJointSetMin(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRotaryLimitJoint).
    pub fn set_max(&self, value: CPFloat) {
        unsafe { cpRotaryLimitJointSetMax(self.to_constraint(), value) }
    }
}

impl Constraint for RotaryLimitJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for RotaryLimitJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `RatchetJoint`.
pub type RatchetJointHandle = Rc<RefCell<RatchetJoint>>;

/// A Ratchet Joint.
pub struct RatchetJoint {
    ptr: *const CPRatchetJoint,
}

impl RatchetJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, phase: CPFloat, ratchet: CPFloat) -> RatchetJoint {
        unsafe {
            RatchetJoint {
                ptr: cpRatchetJointNew(body_a.borrow().ptr, body_b.borrow().ptr, phase, ratchet) as *const CPRatchetJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn angle(&self) -> CPFloat {
        unsafe { cpRatchetJointGetAngle(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn phase(&self) -> CPFloat {
        unsafe { cpRatchetJointGetPhase(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn ratchet(&self) -> CPFloat {
        unsafe { cpRatchetJointGetRatchet(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn set_angle(&self, value: CPFloat) {
        unsafe { cpRatchetJointSetAngle(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn set_phase(&self, value: CPFloat) {
        unsafe { cpRatchetJointSetPhase(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpRatchetJoint).
    pub fn set_ratchet(&self, value: CPFloat) {
        unsafe { cpRatchetJointSetRatchet(self.to_constraint(), value) }
    }
}

impl Constraint for RatchetJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for RatchetJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `GearJoint`.
pub type GearJointHandle = Rc<RefCell<GearJoint>>;

/// A Gear Joint.
pub struct GearJoint {
    ptr: *const CPGearJoint,
}

impl GearJoint {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGearJoint).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, phase: CPFloat, ratio: CPFloat) -> GearJoint {
        unsafe {
            GearJoint {
                ptr: cpGearJointNew(body_a.borrow().ptr, body_b.borrow().ptr, phase, ratio) as *const CPGearJoint
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGearJoint).
    pub fn phase(&self) -> CPFloat {
        unsafe { cpGearJointGetPhase(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGearJoint).
    pub fn ratio(&self) -> CPFloat {
        unsafe { cpGearJointGetRatio(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGearJoint).
    pub fn set_phase(&self, value: CPFloat) {
        unsafe { cpGearJointSetPhase(self.to_constraint(), value) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpGearJoint).
    pub fn set_ratio(&self, value: CPFloat) {
        unsafe { cpGearJointSetRatio(self.to_constraint(), value) }
    }
}

impl Constraint for GearJoint {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for GearJoint {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `SimpleMotor`.
pub type SimpleMotorHandle = Rc<RefCell<SimpleMotor>>;

/// A Simple Motor.
pub struct SimpleMotor {
    ptr: *const CPSimpleMotor,
}

impl SimpleMotor {
    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSimpleMotor).
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, rate: CPFloat) -> SimpleMotor {
        unsafe {
            SimpleMotor {
                ptr: cpSimpleMotorNew(body_a.borrow().ptr, body_b.borrow().ptr, rate) as *const CPSimpleMotor
            }
        }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSimpleMotor).
    pub fn rate(&self) -> CPFloat {
        unsafe { cpSimpleMotorGetRate(self.to_constraint()) }
    }

    /// See [Chipmunk Pin Joint](http://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#ConstraintTypes-cpSimpleMotor).
    pub fn set_rate(&self, value: CPFloat) {
        unsafe { cpSimpleMotorSetRate(self.to_constraint(), value) }
    }

}

impl Constraint for SimpleMotor {
    fn to_constraint(&self) -> *const CPConstraint {
        self.ptr as *const CPConstraint
    }
}

impl Drop for SimpleMotor {
    fn drop(&mut self) {
        unsafe {
            cpConstraintFree(self.to_constraint());
        }
    }
}

/// Reference counted handle to a `Shape`.
pub type ShapeHandle = Rc<RefCell<Box<Shape>>>;

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
