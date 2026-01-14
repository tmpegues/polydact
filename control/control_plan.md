# Control
I'm planning on having my digit be a 3 cable spirob controlled with two sensors. The digit will be controlled by moving a control point in 2d across an equalateral triangle, where the control points at the corners represent maximum curl by one motor. Contect of of the control point with an edge opposite of a corner would represent no curl/slacked cable by that motor.

![](./motor_triangle1.svg)

Current plan is to move the control point with two flex sensors attached to the pinky of the glove. The flex sensors will be placed rougly normal to each other; curling the pinky towards the palm will move the control point up, towards the max of cable 1, curling the digit towards the palm.. Straightening and maybe bending slightly backwards will move the point down towards edge 2,3. Reaching the pinky in the plane of the palm away from the rest of the fingers will move right on the diagram above, which will move the the digit largely, but not completely in the plane of the palm away from the fingers.

The strie control


Aside from basic motions, I plan to include force sensor on the back of the glove that will lock and unlock the digit when pressed. I think a rapid curl towards palm (teleport control point to motor 1 point) and rapid return to neutral (teleport control point to 0,0) would be useful. These could be mapped to a full straighten-then-flex of the index and middle fingers. A "set current pinky position as neutral" would probably also be useful; this would probably be a combination of press-force-sensor and flex index finger.

# Glove and Sensors
Glove will be a basic baseball batting glove. It's light enough and designed for grasping, but importantly has a strap on the wrist, so should work well for fixing the digit base to the hand firmly.

4 total flex + 1 force

back of hand: lock
pinky: move control point
index and middle fingers: teleport to full flex and neutral

Sensor attchment:

* Flex sensors will have two attachment points to the glove each.
1. Fixed attachment either just below or just above the base of the finger.
    * The pin side fo the flex sensor will be firmly attached to the glove here
    * Lateral pinky sensor will connect hand to finger
    * Palm pinky sensor will connect 1st and 2nd links
2. A 3D printed piece with a slot for the free end of the sensor to slide through will be attached across the joint
    * The free side of the flex sensor will be inserted through this slow.
    * This will hopefully allow the end of the flex sensor to move along the finger so that it doesn't flex and bend away from the hand when the finger is straightened.
* The force sensor will be fixed to the back of the glove

# Motors
Motors will be rigidly mounted above the user's wrist. Near the elbow perhaps? Cables will be rounted through PTFE tubing along the arm. Motor 1's tube will attach on the palm side of the hand and will likely be rounted along the underside of the arm.

Questions:
* Tensioning cables when motors are at neutral? Mechanical system or requireing small amount of force/current feedback on the motors at all time?