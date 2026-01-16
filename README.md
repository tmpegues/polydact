# Polydact: a flexible extra digit
* Finger model v1 changes required:
    * Modify hinge angle to allow full curling
    * Add spring to re-straighten finger
    * Proportionalize tip cutoff, make round?
    * Increase cable hole size
* Finger model v2 changes required:
    * 3D printed spring is a little stiff.
    * Make a spring that only connects the base and tip, skips the mid section
    * Widen the cable hole just a little bit more

* 0109
    * Need to read paper shared by Halley
    * Need to read Kris's project
    * Need to start hand mount
* 0113
    * Changed plan to start with 3 cable tentacle.
    * Began settling out plan for sensor glove.
    * Need to figure out how motors and ROS will work
    * Printed 14 bone green digit. ([video](./media/3d_14bead.mp4))
        * Spine hole was 2 mm in CAD, too small for 1.75 mm NinjaFlex.
        * Drilled out spine hole and superglued bones to NinjaFlex spine.
            * Supergluing the bones to the spine seems suboptimal, but it seems to work well enough.
            * Can I maybe cut the bones in half with registration keys if I'm going to need to be super gluing anyway?
            * This would allow me to get a tighter fit on the spine while still perhaps allowing a little bit of flexibity on it

* 0114
    * Printing more bones of yesterday's Polydact
        * Increased spine hole from 2 mm to 2.5 in cad.
        * 14 green + 16 pink = 30 units, 2.5 wraps
    * Continuing to formalize control concept ([detail](./control/overview.md))
    * Reading more about motors ([detail](./control/motor_detail.md))
* 0115
    * Got motors moving usign Dynamixel Wizard, following [Quick Starts](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/quick_start_guide/#quick-start-guide)