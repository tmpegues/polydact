# Control
I'm planning on having Polydact be a 3 cable spirob controlled with two sensors. It will be controlled by moving a control point in 2d across an equalateral triangle, where the control points at the corners represent maximum curl by one motor. Contact of of the control point with an edge opposite of a corner would represent no curl/slacked cable by that motor.

![](./motor_triangle1.svg)

Current plan is to move the control point with two flex sensors attached to the pinky of the glove. The flex sensors will be placed rougly normal to each other; pinky flexion will move the control point up, towards the max of cable 1, flexing the Polydact. Pinky extension will move the point down towards edge 2,3. Pinky abduction will move the point right on the diagram above, which will abduct the digit


Aside from basic motions, I plan to include force sensor on the back of the glove that will lock and unlock the Polydact when pressed. I think a rapid full flex(teleport control point to motor 1 coner) and rapid return to neutral (teleport control point to 0,0) would be useful. These could be mapped to a full extension->full flexion of the index and middle fingers respectively. A "set current pinky position as neutral" would probably also be useful; this would probably be a combination of press-force-sensor and flex index finger.

# Glove and Sensors
The glove will be a basic baseball batting glove. It's light enough and designed for grasping, but importantly has a strap on the wrist, so should work well for fixing the digit base to the hand firmly.

4 total flex sensors + 1 force sensor

back of hand force sensor: freeze control point
pinky: move control point
index and middle fingers: teleport control point to pre-set positions

Sensor attchment:

* Flex sensors will measure angle of either MCP or PIP (palm to finger joint or middle knuckle) have two attachment points to the glove each.
1. Fixed attachment distal to the joint.
    * The pin side fo the flex sensor will be firmly attached to the glove here
    * Pinky abduction sensor will be across MCP
    * Pinky flexion sensor will be across PIP
2. A 3D printed piece with a slot for the free end of the sensor to slide through will be attached across the joint
    * The free end of the flex sensor will be inserted through this slot.
    * This will hopefully allow the end of the flex sensor to move along the finger so that it doesn't flex and bend away from the hand when the finger is straightened.
* The force sensor will be fixed to the back of the glove

# Motors ([details](./motor_detail.md))

Motors will be rigidly mounted above the user's wrist. Near the elbow perhaps? Cables will be rounted through PTFE tubing along the arm. Motor 1's tube will attach on the palm side of the hand and will likely be rounted along the underside of the arm.


# Questions:
* Tensioning cables when motors are at neutral? Mechanical system or requireing small amount of force/current feedback on the motors at all time?