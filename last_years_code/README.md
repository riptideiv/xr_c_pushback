# Last Year's Code

This folder (/last_years_code/riptide/) contains my code from last year that I'm working on adapting over to this year (in /include/). Note that many functions on the robot this year are unique to this year and many functions last year are unique to last year, so we most likely will only be transferring the custom frameworks over (e.g. the auton selector/runner before driver control).

Additionally, we used Lemlib last year and will not be using it this year; I am writing my own odom/PID/pure pursuit library.

TODO:
- [x] Add auton selector to this year's code
  - Added `include/auton.hpp` and `src/auton.cpp` with the auton selector framework
  - Integrated into `src/riptide.cpp` - auton selector initializes on startup
  - Use controller buttons: Y=toggle color, X=next route, B=previous route, D-pad=run for testing, bumpers=confirm selection
- [x] Expand the new odom library to include functions similar to lemlib for PID & Pure Pursuit
  - Added to `include/odom.hpp` and `src/odom.cpp`: `getX()`, `getY()`, `getHeading()`, `setPose()`, `resetPose()`, `distanceTo()`, `angleTo()`
  - Added to `include/pid.hpp` and `src/pid.cpp`: `moveToPoint()`, `turnToHeading()`, `turnToPoint()`, `moveToPose()`, `followPath()` (pure pursuit), `cancelMotion()`, `isMoving()`
  - Parameter structs similar to lemlib: `MoveToPointParams`, `TurnToHeadingParams`, `MoveToPoseParams`
  - Support for async movement with motion cancellation