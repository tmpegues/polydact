I'm planning on having my digit be a 3 cable spirob controlled with two sensors. The digit will be controlled by moving a control point in 2d across an equalateral triangle, where the control points at the corners represent maximum curl by one motor. Contect of of the control point with an edge opposite of a corner would represent no curl/slacked cable by that motor.

![](./motor_triangle1.svg)

Current plan is to move the control point with two flex sensors attached to the pinky of the glove. The flex sensors will be placed rougly normal to each other; curling the pinky towards the palm will move the control point up, towards the max of cable 1, curling the digit towards the palm.. Straightening and maybe bending slightly backwards will move the point down towards edge 2,3. Reaching the pinky in the plane of the palm away from the rest of the fingers will move right on the diagram above, which will move the the digit largely, but not completely in the plane of the palm away from the fingers.


Aside from basic motions, I plan to include force sensor on the back of the glove that will lock and unlock the digit when pressed. I think a rapid curl towards palm (teleport control point to motor 1 point) and rapid return to neutral (teleport control point to 0,0) would be useful. These could be mapped to a full straighten-then-flex of the index and middle fingers. A "set current pinky position as neutral" would probably also be useful; this would probably be a combination of press-force-sensor and flex index finger.

4 total flex + 1 force

back of hand: lock
pinky: move control point
index and middle fingers: teleport to full flex and neutral