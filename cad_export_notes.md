# CAD → MuJoCo export notes

How the Onshape assembly becomes the simulation today, and what to change in CAD so
re-exports stop breaking the code.

## How it works now

1. The Onshape assembly is exported to `full_assembly/urdf/full_assembly.urdf` plus STL
   meshes in `full_assembly/meshes/`.
2. `build_model()` in `src/quadruped_model.py` patches the export:
   - tells MuJoCo where the meshes are and to keep the visible geometry
   - deletes pulleys that weren't mated to anything and float beside the robot
   - recalculates mass from the meshes, because Onshape exported near-zero masses
   - lets robot parts touch the floor but not each other
   - adds the ground and a free joint so the robot isn't bolted to the world
   - turns the 12 revolute joints into position servos modelled on the DS3235SG
     (looked up by their export names, e.g. `fastened_16_1`, in `SERVO_JOINTS`)
3. `src/control_policies/kinematics.py` has joint anchor points, axes and foot points
   copied by hand from the model (`CAD_LEGS`), plus `CAD_BODY_ORIGIN` and
   `FORWARD_IS_CAD_PLUS_Y` to convert them into the robot's own frame.
4. `src/backends/mujoco_backend.py` re-levels the chassis on every reset, because the
   URDF's root part is leg d's foot instead of the chassis.

Every re-export can change the joint names, leg d's sign flips and the measured numbers,
so all of the above has to be re-checked by hand.

## What to fix in CAD

- [ ] **Make the chassis the root part.** Fix the chassis plate (or make it the first
      part) in the assembly so the URDF starts from it. Removes leg d's flipped axes and
      `_level_chassis_on_ground()`.
- [ ] **Define the robot frame in CAD.** Mate connector on the chassis at the centre of
      the four hips, X forward, Z up. Removes `CAD_BODY_ORIGIN` and
      `FORWARD_IS_CAD_PLUS_Y`.
- [ ] **Decide which end is the front.** Design choice, not a measurement: pick based on
      where the Pi, IMU and any camera sit. Currently assumed CAD +y (knees point back).
- [ ] **Name the joints.** Name the 12 revolute mates e.g. `front_left_knee`, and name legs
      by position instead of a/b/c/d. Replaces the `fastened_*` names in `SERVO_JOINTS`.
      (Not yet confirmed which exporter we use or that it keeps mate names.)
- [ ] **Make joint directions consistent.** Positive rotation should mean the same motion
      on every leg (e.g. positive hip pitch swings the foot forward). The CAD pose should
      match the servo centre positions.
- [ ] **Mark the feet.** Mate connector at each foot contact point, so foot positions are
      exported instead of measured by hand.
- [ ] **Clean up before export.** Give parts real materials/masses; mate or suppress the
      loose pulleys.
