# CHSL
prounced (chisel)
https://crates.io/crates/chsl

2D Game Physics Engine supporting Joints/Constraints

```rs
let mut physics_world = PhysicsWorld::new(BoundingBox {
  x: 0.0,
  y: 0.0,
  width: 1240.0,
  height: 880.0,
});

let steps = 1;
loop {
  physics_world.update(delta_time, steps);
}
```

## Add and Remove RigidBodies

```rs
let circle = RigidBody::new_circle(position, rotation, radius, is_static);

//adding
physics_world.add_body("circle ID", circle);

//removing
physics_world.remove_body("circle ID");
```

## Add and Remove Constraints/Joints

```rs
//body is an id
let slide = Constraint::SlideJoint { body, position, angle, strength };

//adding
physics_world.add_constraint("constraint ID", slide);

//removing
physics_world.add_constraint("constraint ID");
```

## Stuffs
this is a shoddily made engine made for fun so use at your own discretion
