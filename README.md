# FuelSim
This single-file mini-library is a lightweight fuel physics simulation. It handles fuel-fuel, fuel-field, fuel-net, fuel-trench, and fuel-robot collisions as well as projectile motion (with optional air drag simulation) and scoring in the hub. There is also included functionality for intakes. It's not a perfect analog of real life, as some of the parameters need to be tuned (namely the coefficients of restitution of various collisions), but it is close enough for simulation purposes. It logs fuel positions to a NetworkTables topic containing an array of `Translation3d`'s.
## Demo
https://github.com/user-attachments/assets/3b0a20c8-bec2-441b-a4a5-fdaee0f9cb8c

## Usage
### Installation
Simply copy over FuelSim.java into your project! If you put it somewhere other than a "utils" folder, make sure to edit the package name at the top of the file.
### Setup
All measurements are in meters. Logging is done automatically when updateSim is called.

Put this directly in Robot.java:
```java
@Override
public void simulationPeriodic() {
    robotContainer.fuelSim.updateSim();
}
```
These are functions you can put in RobotContainer.java (or elsewhere) that should be run on initialization. Don't copy over this entire code block. Instead, copy the individual lines you want. You can see an example of what setup could look like down below.
```java
public FuelSim fuelSim = new FuelSim(String tableKey); // creates a new fuelSim of FuelSim
fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone

// Register a robot for collision with fuel
fuelSim.registerRobot(
        width, // from left to right in meters
        length, // from front to back in meters
        bumperHeight, // from floor to top of bumpers in meters
        poseSupplier, // Supplier<Pose2d> of robot pose
        fieldSpeedsSupplier); // Supplier<ChassisSpeeds> of field-centric chassis speeds

// Register an intake to remove fuel from the field as a rectangular bounding box
fuelSim.registerIntake(
        minX, maxX, minY, maxY, // robot-centric coordinates for bounding box in meters
        shouldIntakeSupplier, // (optional) BooleanSupplier for whether the intake should be active at a given moment
        callback); // (optional) Runnable called whenever a fuel is intaked

fuelSim.setSubticks(int subticks); // sets the number of physics iterations to perform per 20ms loop. Default = 5

fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)
fuelSim.stop(); // stops the simulation running (updateSim will do nothing until start is called again)

fuelSim.enableAirResistance() // an additional drag force will be applied to fuel in physics update step
```
### While Running
These are methods you can call while the program is running.
```java
fuelSim.stepSim(); // steps the simulation forward by 20ms, regardless of start/stop state
fuelSim.spawnFuel(Translation3d pos, Translation3d vel); // spawns a fuel with a given position and velocity (both field centric, represented as vectors by Translation3d)
fuelSim.launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight); // Spawns a fuel onto the field at the robot's position with a specified launch velocity and angles, accounting for robot movement (robot must be registered)
fuelSim.clearFuel(); // clears all fuel from the field

FuelSim.Hub.BLUE_HUB.getScore(); // get number of fuel scored in blue hub
FuelSim.Hub.RED_HUB.getScore(); // get number of fuel scored in red hub
FuelSim.Hub.[BLUE/RED]_HUB.resetScore(); // resets the score of the blue/red hub

```
## Example Usage (as seen in demo)
In RobotContainer.java (to be called in constructor) (fuelSim was declared previously):
```java
private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
}

private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
    fuelSim.registerRobot(
            Dimensions.FULL_WIDTH.in(Meters),
            Dimensions.FULL_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            drive::getPose,
            drive::getFieldSpeeds);
    fuelSim.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).in(Meters),
            () -> intake.isRightDeployed() && ableToIntake.getAsBoolean(),
            intakeCallback);
    fuelSim.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            () -> intake.isLeftDeployed() && ableToIntake.getAsBoolean(),
            intakeCallback);
}
```

In TurretIOSim.java (`fuelSim` was provided in the constructor):
```java
public boolean canIntake() {
    return fuelStored < CAPACITY;
}

public void intakeFuel() {
    fuelStored++;
}

// called repeatedly
public void launchFuel() {
    if (fuelStored == 0) return;
    fuelStored--;

    fuelSim.launchFuel(
            TurretCalculator.angularToLinearVelocity(flywheelGoal, FLYWHEEL_RADIUS),
            hoodAngle,
            turnPosition,
            ROBOT_TO_TURRET_TRANSFORM.getMeasureZ());
}
```
# Contributing
Contributions are welcome! This project generally follows the [Palantir Java format](https://github.com/palantir/palantir-java-format). The specifics of PJF aren't super important but please make sure you follow these guidelines:
- Don't apply any auto-formatter aside from PJF
- Use 4-space indentations
- Try and keep to the style of the rest of the code
- Keep the file self-contained. No references to external files aside from standard WPILib and AdvantageKit.
- Document all public methods and any non-obvious private methods or variabels. Use javadoc docstrings.
- Test all code before making a pull request

Thank you!
