## To Do ##
- Update shuffleboard with code with two tabs
- Change pipeline based on cargo vs hatch modes and proximity
- Tune auto aim functions
- Make sure that lights are turning off during init
- Program Blinkin LEDs - http://www.revrobotics.com/rev-11-1105/
- Test Declan's auton
- Make an easier way to get softMA

## DONE ##
- Move defense mode to where ever it should be
- Add winch potentiometer and position settings
- Update arm positions for kcap
- Test and update small winch Pot values and kp and max speed

## Others ##
- Update pipelines for hatch and cargo and copy to each camera
- Update shuffleboard

## LIMELIGHT TUNING ##
- Single target W/H Ratio: 0-0.7500 (or 0 to 1)
- Dual target W/H Ratio: 0-0.7500 (or 0 to 1) (for some reason the ratio meaning is flipped for single/dual targets)
- Hue: 75-100
- Saturation: 175-255
- Value: 225-255
## Single Target ##
- Area: .5-15
- Fullness: 50-100
- W/H Ratio: 0-1
- Direction Filter: None
- Contour Simplification: 5
- Target Grouping: Single Target
- Intersection Filter: Vert-Above
## Dual Target ##
- Area: 0.0050-15
- Fullness: 50-100
- W/H Ratio: 0-1
- Direction Filter: None
- Contour Simplification: 5
- Target Grouping: Dual Target
- Intersection Filter: None
## Other Tuning Notes ##
- Erosion will slightly erode the result of an HSV threshold. This is useful if many objects are passing through a tuned HSV threshold.
- Dilation will slightly inflate the result of an HSV threshold. Use this to patch holes in thresholding results.
## Pipelines ##
- 0: Dual target center
- 1: Dual target right contour
- 2: Dual target left contour
- 3: Single target
## SKEW ##
- Skew from the left goes up from 0 to -90
- Skew from the right goes down from -90 to 0
## Corner Values ##
- Values go clockwise from bottom left of bounding box
