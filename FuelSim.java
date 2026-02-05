package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim {

    private static final double PERIOD = 0.02;
    private static int subticks = 5;

    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81);
    private static final double AIR_DENSITY = 1.2041;
    private static final double FIELD_COR = Math.sqrt(22.0 / 51.5);
    private static final double FUEL_COR = 0.5;
    private static final double NET_COR = 0.2;
    private static final double ROBOT_COR = 0.1;

    private static final double FUEL_RADIUS = 0.075;
    private static final double FIELD_LENGTH = 16.51;
    private static final double FIELD_WIDTH = 8.04;

    private static final double TRENCH_WIDTH = 1.265;
    private static final double TRENCH_BLOCK_WIDTH = 0.305;
    private static final double TRENCH_HEIGHT = 0.565;
    private static final double TRENCH_BAR_HEIGHT = 0.102;
    private static final double TRENCH_BAR_WIDTH = 0.152;

    private static final double FRICTION = 0.1;
    private static final double FUEL_MASS = 0.448 * 0.45392;
    private static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    private static final double DRAG_COF = 0.47;
    private static final double DRAG_FORCE_FACTOR =
            0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    private static final double CELL_SIZE = 0.25;
    private static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    private static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    private static final ArrayList<List<Translation3d>> completedTraj = new ArrayList<>();

    private static FuelSim instance;

    @SuppressWarnings("unchecked")
    private final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

    private final ArrayList<Fuel> fuels = new ArrayList<>();
    private final ArrayList<SimIntake> intakes = new ArrayList<>();

    private boolean running;
    private boolean simulateAirResistance;

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotFieldSpeedsSupplier;

    private double robotWidth;
    private double robotLength;
    private double bumperHeight;

    public static FuelSim getInstance() {
        if (instance == null) {
            instance = new FuelSim();
        }
        return instance;
    }

    private FuelSim() {
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j] = new ArrayList<>();
            }
        }
    }

    public void start() {
        running = true;
    }

    public void stop() {
        running = false;
    }

    public void enableAirResistance() {
        simulateAirResistance = true;
    }

    public void setSubticks(int subticks) {
        FuelSim.subticks = subticks;
    }

    public void clearFuel() {
        fuels.clear();
    }

    public void updateSim() {
        if (!running) {
            return;
        }
        stepSim();
    }

    public void stepSim() {
        for (int sub = 0; sub < subticks; sub++) {

            for (int i = 0; i < fuels.size(); i++) {
                Fuel fuel = fuels.get(i);
                fuel.update();

                if (fuel.finished) {
                    completedTraj.add(fuel.traj);
                    if (completedTraj.size() > 1) {
                        completedTraj.remove(0);
                    }
                    fuels.remove(i--);
                }
            }

            handleFuelCollisions(fuels);

            if (robotPoseSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
            }
        }

        logFuels();
    }

    public void logFuels() {
        Logger.recordOutput(
                "Fuel Simulation/Fuels",
                fuels.stream().map(f -> f.pos).toArray(Translation3d[]::new));

        ArrayList<Translation3d> meanTraj = computeMeanTrajectory();
        Logger.recordOutput(
                "Fuel Simulation/Mean Trajectory",
                meanTraj.toArray(new Translation3d[0]));
    }

    public ArrayList<Translation3d> computeMeanTrajectory() {
        if (completedTraj.isEmpty()) {
            return new ArrayList<>();
        }

        int minLength =
                completedTraj.stream().mapToInt(List::size).min().orElse(0);

        ArrayList<Translation3d> mean = new ArrayList<>();
        int maxPoints = 15;
        int step = Math.max(minLength / maxPoints, 1);

        for (int i = 0; i < minLength && mean.size() < maxPoints; i += step) {
            double x = 0;
            double y = 0;
            double z = 0;

            for (List<Translation3d> traj : completedTraj) {
                Translation3d p = traj.get(i);
                x += p.getX();
                y += p.getY();
                z += p.getZ();
            }

            int n = completedTraj.size();
            mean.add(new Translation3d(x / n, y / n, z / n));
        }

        return mean;
    }

    public void spawnFuel(Translation3d pos, Translation3d vel) {
        fuels.add(new Fuel(pos, vel));
    }

    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {

        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
        this.robotPoseSupplier = poseSupplier;
        this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    public void launchFuel(
            LinearVelocity launchVelocity,
            Angle hoodAngle,
            Angle turretYaw,
            Distance launchHeight) {

        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) {
            throw new IllegalStateException("Robot must be registered before launching fuel.");
        }

        Pose3d launchPose =
                new Pose3d(robotPoseSupplier.get())
                        .plus(
                                new Transform3d(
                                        new Translation3d(
                                                Meters.zero(), Meters.zero(), launchHeight),
                                        Rotation3d.kZero));

        ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();

        double horizontal =
                Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double vertical =
                Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);

        double xVel =
                horizontal * Math.cos(turretYaw.in(Radians)) + speeds.vxMetersPerSecond;
        double yVel =
                horizontal * Math.sin(turretYaw.in(Radians)) + speeds.vyMetersPerSecond;

        spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, vertical));
    }


    private class Fuel {

        private Translation3d pos;
        private Translation3d vel;
        private boolean finished;
        private final List<Translation3d> traj = new ArrayList<>();

        private Fuel(Translation3d pos, Translation3d vel) {
            this.pos = pos;
            this.vel = vel;
        }

        private Fuel(Translation3d pos) {
            this(pos, new Translation3d());
        }

        private void update() {
            if (!finished) {
                traj.add(pos);
            }

            pos = pos.plus(vel.times(PERIOD / subticks));

            if (pos.getZ() > FUEL_RADIUS) {
                Translation3d gravityForce = GRAVITY.times(FUEL_MASS);
                Translation3d dragForce = new Translation3d();

                if (simulateAirResistance) {
                    double speed = vel.getNorm();
                    if (speed > 1e-6) {
                        dragForce = vel.times(-DRAG_FORCE_FACTOR * speed);
                    }
                }

                Translation3d accel =
                        gravityForce.plus(dragForce).div(FUEL_MASS);
                vel = vel.plus(accel.times(PERIOD / subticks));
            }

            if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                vel = new Translation3d(vel.getX(), vel.getY(), 0.0);
                vel = vel.times(1.0 - FRICTION * PERIOD / subticks);
            }

            if (pos.getZ() <= FUEL_RADIUS && Math.abs(vel.getZ()) <= 3.0) {
                finished = true;
            }

            handleFieldCollisions();
        }

        private void handleFieldCollisions() {
            handleHubCollisions(Hub.BLUE_HUB);
            handleHubCollisions(Hub.RED_HUB);
        }

        private void handleHubCollisions(Hub hub) {
            if (hub.handleHubInteraction(this)) {
                finished = true;
            }

            hub.fuelCollideSide(this);

            double netCollision = hub.fuelHitNet(this);
            if (netCollision != 0.0) {
                pos = pos.plus(new Translation3d(netCollision, 0, 0));
                vel =
                        new Translation3d(
                                -vel.getX() * NET_COR,
                                vel.getY() * NET_COR,
                                vel.getZ());
            }
        }

        private void addImpulse(Translation3d impulse) {
            vel = vel.plus(impulse);
        }
    }


    public static class Hub {

        public static final Hub BLUE_HUB =
                new Hub(
                        new Translation2d(4.61, FIELD_WIDTH / 2.0),
                        new Translation3d(5.3, FIELD_WIDTH / 2.0, 0.89),
                        1);

        public static final Hub RED_HUB =
                new Hub(
                        new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2.0),
                        new Translation3d(
                                FIELD_LENGTH - 5.3, FIELD_WIDTH / 2.0, 0.89),
                        -1);

        private static final double ENTRY_HEIGHT = 1.83;
        private static final double ENTRY_RADIUS = 0.56;

        private static final double SIDE = 1.2;
        private static final double NET_HEIGHT_MAX = 3.057;
        private static final double NET_HEIGHT_MIN = 1.5;
        private static final double NET_OFFSET = SIDE / 2.0 + 0.261;
        private static final double NET_WIDTH = 1.484;

        private final Translation2d center;
        private final Translation3d exit;
        private final int exitVelXMult;

        private int score;

        private Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        private boolean handleHubInteraction(Fuel fuel) {
            if (didFuelScore(fuel)) {
                fuel.pos = exit;
                fuel.vel = getDispersalVelocity();
                score++;
                return true;
            }
            return false;
        }

        private boolean didFuelScore(Fuel fuel) {
            return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
                    && fuel.pos.getZ() <= ENTRY_HEIGHT
                    && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ()
                            > ENTRY_HEIGHT;
        }

        private Translation3d getDispersalVelocity() {
            return new Translation3d(
                    exitVelXMult * (Math.random() + 0.1) * 1.5,
                    Math.random() * 2.0 - 1.0,
                    0.0);
        }

        public int getScore() {
            return score;
        }

        public void resetScore() {
            score = 0;
        }

        private void fuelCollideSide(Fuel fuel) {
            fuelCollideRectangle(
                    fuel,
                    new Translation3d(
                            center.getX() - SIDE / 2.0,
                            center.getY() - SIDE / 2.0,
                            0.0),
                    new Translation3d(
                            center.getX() + SIDE / 2.0,
                            center.getY() + SIDE / 2.0,
                            ENTRY_HEIGHT - 0.1));
        }

        private double fuelHitNet(Fuel fuel) {
            if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) {
                return 0.0;
            }

            if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2.0
                    || fuel.pos.getY() < center.getY() - NET_WIDTH / 2.0) {
                return 0.0;
            }

            double netX = center.getX() + NET_OFFSET * exitVelXMult;

            if (fuel.pos.getX() > netX) {
                return Math.max(0.0, netX - (fuel.pos.getX() - FUEL_RADIUS));
            } else {
                return Math.min(0.0, netX - (fuel.pos.getX() + FUEL_RADIUS));
            }
        }
    }


    private class SimIntake {

        private final double xMin;
        private final double xMax;
        private final double yMin;
        private final double yMax;

        private final BooleanSupplier ableToIntake;
        private final Runnable callback;

        private SimIntake(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable callback) {

            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = callback;
        }

        private boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) {
                return false;
            }

            Translation2d relativePos =
                    new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                            .relativeTo(robotPose)
                            .getTranslation();

            boolean inside =
                    relativePos.getX() >= xMin
                            && relativePos.getX() <= xMax
                            && relativePos.getY() >= yMin
                            && relativePos.getY() <= yMax;

            if (inside) {
                callback.run();
            }

            return inside;
        }
    }

    
    private void handleFuelCollisions(ArrayList<Fuel> fuels) {
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j].clear();
            }
        }

        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(fuel);
            }
        }

        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i < 0 || i >= GRID_COLS || j < 0 || j >= GRID_ROWS) {
                        continue;
                    }

                    for (Fuel other : grid[i][j]) {
                        if (fuel != other
                                && fuel.pos.getDistance(other.pos) < FUEL_RADIUS * 2.0
                                && fuel.hashCode() < other.hashCode()) {
                            handleFuelCollision(fuel, other);
                        }
                    }
                }
            }
        }
    }

    private static void handleFuelCollision(Fuel a, Fuel b) {
        Translation3d normal = a.pos.minus(b.pos);
        double distance = normal.getNorm();

        if (distance == 0.0) {
            normal = new Translation3d(1, 0, 0);
            distance = 1.0;
        }

        normal = normal.div(distance);

        double impulse =
                0.5 * (1.0 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));

        double penetration = FUEL_RADIUS * 2.0 - distance;

        a.pos = a.pos.plus(normal.times(penetration / 2.0));
        b.pos = b.pos.minus(normal.times(penetration / 2.0));

        a.addImpulse(normal.times(impulse));
        b.addImpulse(normal.times(-impulse));
    }

    private void handleRobotCollisions(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();
        ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();

        Translation2d robotVel =
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        for (Fuel fuel : fuels) {
            handleRobotCollision(fuel, robot, robotVel);
        }
    }

    private void handleRobotCollision(
            Fuel fuel, Pose2d robot, Translation2d robotVel) {

        Translation2d relativePos =
                new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                        .relativeTo(robot)
                        .getTranslation();

        if (fuel.pos.getZ() > bumperHeight) {
            return;
        }

        double dx = robotLength / 2.0 + FUEL_RADIUS - Math.abs(relativePos.getX());
        double dy = robotWidth / 2.0 + FUEL_RADIUS - Math.abs(relativePos.getY());

        if (dx <= 0.0 || dy <= 0.0) {
            return;
        }

        Translation2d normal =
                dx < dy
                        ? new Translation2d(Math.signum(relativePos.getX()), 0.0)
                        : new Translation2d(0.0, Math.signum(relativePos.getY()));

        normal = normal.rotateBy(robot.getRotation());

        fuel.pos = fuel.pos.plus(new Translation3d(normal.times(Math.min(dx, dy))));

        double relVel = fuel.vel.toTranslation2d().dot(normal);
        if (relVel < 0.0) {
            fuel.addImpulse(
                    new Translation3d(normal.times(-relVel * (1.0 + ROBOT_COR))));
        }

        double robotRel = robotVel.dot(normal);
        if (robotRel > 0.0) {
            fuel.addImpulse(new Translation3d(normal.times(robotRel)));
        }
    }

    private void handleIntakes(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();

        for (SimIntake intake : intakes) {
            for (int i = 0; i < fuels.size(); i++) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    fuels.remove(i--);
                }
            }
        }
    }

    private static void fuelCollideRectangle(
            Fuel fuel, Translation3d start, Translation3d end) {

        if (fuel.pos.getZ() > end.getZ() + FUEL_RADIUS
                || fuel.pos.getZ() < start.getZ() - FUEL_RADIUS) {
            return;
        }

        double left = start.getX() - FUEL_RADIUS - fuel.pos.getX();
        double right = fuel.pos.getX() - end.getX() - FUEL_RADIUS;
        double top = fuel.pos.getY() - end.getY() - FUEL_RADIUS;
        double bottom = start.getY() - FUEL_RADIUS - fuel.pos.getY();

        if (left > 0 || right > 0 || top > 0 || bottom > 0) {
            return;
        }

        if (Math.abs(left) < Math.abs(right)) {
            fuel.pos = fuel.pos.plus(new Translation3d(left, 0, 0));
            fuel.vel =
                    fuel.vel.plus(
                            new Translation3d(-(1.0 + FIELD_COR) * fuel.vel.getX(), 0, 0));
        } else {
            fuel.pos = fuel.pos.plus(new Translation3d(-right, 0, 0));
            fuel.vel =
                    fuel.vel.plus(
                            new Translation3d(-(1.0 + FIELD_COR) * fuel.vel.getX(), 0, 0));
        }
    }
}
