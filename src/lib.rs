
extern crate libc;

pub mod ffi;

use std::rc::*;
use std::cell::*;
use std::marker::{PhantomData};

use ffi::*;

// pub type ShapeHandle = Rc<RefCell<Box<Shape>>>;

// Constructor is not public else the underlying reference can be droped/freed
// by the use by wrapping in Owned and letting it go out of scope.

// Bindings to the CHipmunk 7.0.1 library

// Higher level bindings looks to stay true

// Space

pub trait ChipmunkRsTypes {
    type Vect;
    fn to_vect(cp_vect: CPVect) -> Self::Vect;
    fn to_cp_vect(vect: &Self::Vect) -> CPVect;
}

type BodyHandle<T> = Rc<RefCell<Body<T>>>;
type ShapeHandle<T> = Rc<RefCell<Box<Shape<T>>>>;

pub struct Space<T: ChipmunkRsTypes> {
    ptr: *const CPSpace,
    bodies: Vec<BodyHandle<T>>,
    shapes: Vec<ShapeHandle<T>>
}

impl<T: ChipmunkRsTypes> Space<T> {
    pub fn new() -> Space<T> {
        Space {
            ptr: unsafe { cpSpaceNew() },
            bodies: Vec::new(),
            shapes: Vec::new()
        }
    }

    // Settings
    pub fn gravity(&self) -> T::Vect {
        unsafe { T::to_vect(cpSpaceGetGravity(self.ptr)) }
    }

    pub fn is_locked(&self) -> bool {
        unsafe { cpSpaceIsLocked(self.ptr) }
    }

    pub fn current_time_step(&self) -> f64 {
        unsafe { cpSpaceGetCurrentTimeStep(self.ptr) }
    }

    pub fn iterations(&self) -> i32 {
        unsafe { cpSpaceGetIterations(self.ptr) }
    }

    pub fn damping(&self) -> f64 {
        unsafe { cpSpaceGetDamping(self.ptr) }
    }

    pub fn idle_speed_threshold(&self) -> f64 {
        unsafe { cpSpaceGetIdleSpeedThreshold(self.ptr) }
    }

    pub fn sleep_time_threshold(&self) -> f64 {
        unsafe { cpSpaceGetSleepTimeThreshold(self.ptr) }
    }

    pub fn collision_slop(&self) -> f64 {
        unsafe { cpSpaceGetCollisionSlop(self.ptr) }
    }

    pub fn collision_bias(&self) -> f64 {
        unsafe { cpSpaceGetCollisionBias(self.ptr) }
    }

    pub fn collision_persistence(&self) -> u32 {
        unsafe { cpSpaceGetCollisionPersistence(self.ptr) }
    }

    // TODO the result should not be deconstructed. the CPSpace handles that.
    // pub fn static_body(&self) -> &Body {
    //     unsafe { &Body { ptr: cpSpaceGetStaticBody(self.ptr) } }
    // }

    pub fn set_gravity(&self, g: T::Vect) {
        unsafe { cpSpaceSetGravity(self.ptr, T::to_cp_vect(&g)) };
    }

    pub fn set_iterations(&mut self, value: i32)
    {
        unsafe { cpSpaceSetIterations(self.ptr, value); }
    }

    pub fn set_damping(&mut self, value: f64)
    {
        unsafe { cpSpaceSetDamping(self.ptr, value); }
    }

    pub fn set_idle_speed_threshold(&mut self, value: f64)
    {
        unsafe { cpSpaceSetIdleSpeedThreshold(self.ptr, value); }
    }

    pub fn set_sleep_time_threshold(&mut self, value: f64)
    {
        unsafe { cpSpaceSetSleepTimeThreshold(self.ptr, value); }
    }

    pub fn set_collision_slop(&mut self, value: f64)
    {
        unsafe { cpSpaceSetCollisionSlop(self.ptr, value); }
    }

    pub fn set_collision_bias(&mut self, value: f64)
    {
        unsafe { cpSpaceSetCollisionBias(self.ptr, value); }
    }

    pub fn set_collision_persistence(&mut self, value: u32)
    {
        unsafe { cpSpaceSetCollisionPersistence(self.ptr, value); }
    }

    // Operations

    pub fn step(&mut self, dt: f64) {
        unsafe { cpSpaceStep(self.ptr, dt) }
    }

    // TODO take Shape instead of a Box<Shape>
    pub fn add_shape(&mut self, shape: Box<Shape<T>>) -> ShapeHandle<T> {
        // unsafe { cpSpaceAddBody(self.ptr, shape.body().ptr); }
        unsafe { cpSpaceAddShape(self.ptr, shape.to_shape()); }
        let handle = Rc::new(RefCell::new(shape));
        self.shapes.push(handle.clone());
        handle
    }

    // TODO do we want to allow this? or just add_shape?
    pub fn add_body(&mut self, body: Body<T>) -> BodyHandle<T>{
        unsafe { cpSpaceAddBody(self.ptr, body.ptr); }
        let handle = Rc::new(RefCell::new(body));
        self.bodies.push(handle.clone());
        handle
    }

    // TODO remove items from Space so that they get dropped.
    //pub fn remove_shape<S: Shape<T>>(&mut self, shape: S) {
    //    unsafe { cpSpaceRemoveShape(self.ptr, shape.to_shape()) }
    //}
    //
    //pub fn remove_body(&mut self, body: Body<T>) {
    //    unsafe { cpSpaceRemoveBody(self.ptr, body.ptr) }
    //}

    pub fn contains_shape(&self, shape: ShapeHandle<T>) -> bool {
        unsafe { cpSpaceContainsShape(self.ptr, shape.borrow().to_shape()) }
    }

    pub fn contains_body(&self, body: BodyHandle<T>) -> bool {
        unsafe { cpSpaceContainsBody(self.ptr, body.borrow().ptr) }
    }

    // cpConstraint *cpSpaceAddConstraint(&self, constraint: cpConstraint)
    // pub fn remove_constraint(&self, constraint: cpConstraint);
    // cpBool cpSpaceContainsConstraint(&self, constraint: cpConstraint)

    //// Misc.
    //pub fn reindex_shape<S: Shape<T>>(&mut self, shape: S) {
    //    unsafe { cpSpaceReindexShape(self.ptr, shape.to_shape()); }
    //}
    //pub fn reindex_shapes_for_body(&mut self, body: Body<T>) {
    //    unsafe { cpSpaceReindexShapesForBody(self.ptr, body.ptr); }
    //}
    //pub fn reindex_static(&mut self) {
    //    unsafe { cpSpaceReindexStatic(self.ptr, ); }
    //}
    // iterators? body/shape/constraint
}

impl<T: ChipmunkRsTypes> Drop for Space<T> {
    fn drop(&mut self) {
        unsafe { cpSpaceFree(self.ptr); }

        // TODO free all resources (e.g. bodies, shapes, constraints, etc).
    }
}


// Body

pub struct Body<T: ChipmunkRsTypes> {
    _marker: PhantomData<T>,
    ptr: *const CPBody
}

impl<T: ChipmunkRsTypes> Body<T> {
    pub fn new_dynamic(m: f64, i: f64) -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNew(m, i)
        } }
    }

    pub fn new_static() -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNewStatic()
        } }
    }

    pub fn new_kinematic() -> Body<T> {
        unsafe { Body {
            _marker: PhantomData,
            ptr: cpBodyNewKinematic()
        } }
    }

    pub fn position(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetPosition(self.ptr)) }
    }

    pub fn center_of_gravity(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetCenterOfGravity(self.ptr)) }
    }

    pub fn velocity(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetVelocity(self.ptr)) }
    }

    pub fn force(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetForce(self.ptr)) }
    }

    pub fn angle(&self) -> f64 {
        unsafe { cpBodyGetAngle(self.ptr) }
    }

    pub fn angular_velocity(&self) -> f64 {
        unsafe { cpBodyGetAngularVelocity(self.ptr) }
    }

    pub fn torque(&self) -> f64 {
        unsafe { cpBodyGetTorque(self.ptr) }
    }

    pub fn rotation(&self) -> T::Vect {
        unsafe { T::to_vect(cpBodyGetRotation(self.ptr)) }
    }

    // pub fn space(&self) -> Option<Space> {
    //     unsafe { cpBodyGetSpace(self.ptr).map(|ptr| Space { ptr: ptr }) }
    // }

    pub fn set_mass(&mut self, m: f64) {
        unsafe { cpBodySetMass(self.ptr, m); }
    }

    pub fn set_moment(&mut self, i: f64) {
        unsafe { cpBodySetMoment(self.ptr, i); }
    }

    pub fn set_position(&mut self, pos: T::Vect) {
        unsafe { cpBodySetPosition(self.ptr, T::to_cp_vect(&pos)); }
    }

    pub fn set_center_of_gravity(&mut self, cog: T::Vect) {
        unsafe { cpBodySetCenterOfGravity(self.ptr, T::to_cp_vect(&cog)); }
    }

    pub fn set_velocity(&mut self, value: T::Vect) {
        unsafe { cpBodySetVelocity(self.ptr, T::to_cp_vect(&value)); }
    }

    pub fn set_force(&mut self, value: T::Vect) {
        unsafe { cpBodySetForce(self.ptr, T::to_cp_vect(&value)); }
    }

    pub fn set_angle(&mut self, a: f64) {
        unsafe { cpBodySetAngle(self.ptr, a); }
    }

    pub fn set_angular_velocity(&mut self, value: f64) {
        unsafe { cpBodySetAngularVelocity(self.ptr, value); }
    }

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

// Shape

pub trait Shape<T: ChipmunkRsTypes> {
    fn to_shape(&self) -> *const CPShape;
    fn body(&self) -> BodyHandle<T>;
    // fn space(&self) -> &Space<T>;
    // fn space_mut(&mut self) -> &mut Space<T>;

    fn elasticity(&self) -> f64 {
        unsafe { cpShapeGetElasticity(self.to_shape()) }
    }

    fn friction(&self) -> f64 {
        unsafe { cpShapeGetFriction(self.to_shape()) }
    }

    fn set_body(&mut self, body: Body<T>) {
        unsafe { cpShapeSetBody(self.to_shape(), body.ptr); }
    }

    fn set_elasticity(&mut self, value: f64) {
        unsafe { cpShapeSetElasticity(self.to_shape(), value); }
    }

    fn set_friction(&mut self, value: f64) {
        unsafe { cpShapeSetFriction(self.to_shape(), value); }
    }
}

// CircleShape

pub struct CircleShape<T: ChipmunkRsTypes> {
    ptr: *const CPCircleShape,
    body: BodyHandle<T>
}

impl<T: ChipmunkRsTypes> CircleShape<T> {
    pub fn new(body: BodyHandle<T>, radius: f64, offset: T::Vect) -> CircleShape<T> {
        unsafe {
            CircleShape {
                ptr: cpCircleShapeNew(body.borrow().ptr, radius, T::to_cp_vect(&offset)),
                body: body.clone()
            }
        }
    }

    pub fn offset(&self) -> T::Vect {
        unsafe { T::to_vect(cpCircleShapeGetOffset(self.ptr)) }
    }

    pub fn radius(&self) -> f64 {
        unsafe { cpCircleShapeGetRadius(self.ptr) }
    }
}

impl<T: ChipmunkRsTypes> Shape<T> for CircleShape<T> {
    fn to_shape(&self) -> *const CPShape {
        self.ptr as *const CPShape
    }

    fn body(&self) -> BodyHandle<T> {
        self.body.clone()
    }

    // fn space(&self) -> Option<&Body<T>> {
    //     &self.space
    // }
    //
    // fn space_mut(&mut self) -> Option<&mut Body<T>> {
    //     &mut self.space
    // }
}

impl<T: ChipmunkRsTypes> Drop for CircleShape<T> {
    fn drop(&mut self) {
        unsafe { cpShapeFree(self.to_shape()); }
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
