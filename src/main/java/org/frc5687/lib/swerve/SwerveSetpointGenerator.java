package org.frc5687.lib.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.frc5687.lib.math.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This is 254s SwerveSetpointGenerator but modified to use WPILibs Geometry classes and an
 * extension to the regula falsi method using Illinois or ITP for root finding.
 */
public class SwerveSetpointGenerator {

    public static class KinematicLimits {
        public double maxDriveVelocity;
        public double maxDriveAcceleration;
        public double maxSteeringVelocity;
    }

    private final SwerveDriveKinematics _kinematics;
    private final Translation2d[] _modulePositions;
    private static final double EPSILON = 1e-9;

    public SwerveSetpointGenerator(
            final SwerveDriveKinematics kinematics, final Translation2d[] modulePositions) {
        _modulePositions = modulePositions;
        _kinematics = kinematics;
    }

    protected static boolean epsilonEquals(double a, double b) {
        return (a - EPSILON <= b) && (a + EPSILON >= b);
    }

    /**

     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive direction).

     *

     * @param prevToGoal The rotation from the previous state to the goal state (i.e. prev.inverse().rotateBy(goal)).

     * @return True if the shortest path to achieve this rotation involves flipping the drive direction.

     */
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    /**
     * modulos an angle to the ref-pi to ref+pi range
     * @param ref the "zero point"
     * @param angle the angle to plug in
     * @return the resulting angle
     */
    public static double unwrapAngle(double ref, double angle) {
        return (angle - ref + Math.PI) % (2 * Math.PI) + ref - Math.PI;
    }

    @FunctionalInterface
    public interface Function2d {
        double f(double x, double y);
    }

    /**
     * Assume function `func` is continuous and that func.f(x_0, y_0) and func.f(x_1, y_1) have different signs. Linearly interpolate between the two input points, looking for the zero, and return the interpolation constant.
     * @see https://www.youtube.com/watch?v=pg1I8AG59Ik
     * @param func function takes in (x,y) outputs a value.
     * @param x_0 x value at s=0
     * @param y_0 y value at s=0
     * @param x_1 x value at s=1
     * @param y_1 y value at s=1
     * @param iterations_left
     * @return 's'. The interpolation constant. Equals 0 when the zero is at (x_0, y_0), equals 1 when the zero is at (x_1, y_1), and equals 0.5 when the zero is at ((x_0+x_1)/2, (y_0,y_1)/2)
     */
    private static double findRootRegula(
            Function2d func,
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            int iterations_left) {
        double f_0 = func.f(x_0, y_0);
        double f_1 = func.f(x_1, y_1);
        if (epsilonEquals(f_0, 0)) {
            return 0.0;
        } else if (epsilonEquals(f_1, 0)) {
            return 1.0;
        } else if (iterations_left < 0) {
            System.err.println("Failed to reach target error "+EPSILON+", error was "+(f_1-f_0));
            return 0.5;
        }
        if (Math.signum(f_0) == Math.signum(f_1)) {
            System.err.println("Cannot find root if f_0 and f_1 have the same sign. f_0 = "+f_0+" and f_1 = "+f_1);
        }
        
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRootRegula(func, x_guess, y_guess, x_1, y_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRootRegula(func, x_0, y_0, x_guess, y_guess, iterations_left - 1);
        }
    }

    public static double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            int iterations_left) {
        return findRootRegula(func, x_0, y_0, x_1, y_1, iterations_left);
    }

    protected double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_deviation,
            int max_iterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        // return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
        return findRoot(func, x_0, y_0, x_1, y_1, max_iterations); // FIXME NOT SURE IF THIS WORKS
    }

    /**
     * @param x_0 vx of the swerve module at s=0
     * @param y_0 vy of the swerve module at s=0
     * @param x_1 vx of the swerve module at s=1
     * @param y_1 vy of the swerve module at s=1
     * @param max_vel_step
     * @param max_iterations
     * @return max s 0-1
     */
    protected double findDriveMaxS(
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            double max_vel_step,
            int max_iterations) {
        double f_0 = Math.hypot(x_0, y_1);
        double f_1 = Math.hypot(x_0, y_1);
        double diff = f_1 - f_0; // difference in speed at s=1 vs. s=0
        if (Math.abs(diff) <= max_vel_step) {
            // If accelerating less than the max acceleration this frame, you can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step; // maximum speed of the swerve module after this frame (given the acceleration limit)
        Function2d func = (x, y) -> Math.hypot(x, y) - offset; // returns the amount faster than the max speed that a vx and vy are
        return findRoot(func, x_0, y_0, x_1, y_1, max_iterations);
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
     *     iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path
     *     following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
     *     desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final KinematicLimits limits,
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState,
            double dt) {
        final Translation2d[] modules = _modulePositions;

        SwerveModuleState[] desiredModuleState = _kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.maxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity);
            desiredState = _kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
        // just use the previous angle.
        boolean need_to_steer = true;
        if (GeometryUtil.toTwist2d(desiredState).equals(GeometryUtil.IDENTITY)) { // FIXME epsilonEquals might be required here.
            need_to_steer = false;
            for (int i = 0; i < modules.length; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = new double[modules.length];
        double[] prev_vy = new double[modules.length];
        Rotation2d[] prev_heading = new Rotation2d[modules.length];
        double[] desired_vx = new double[modules.length];
        double[] desired_vy = new double[modules.length];
        Rotation2d[] desired_heading = new Rotation2d[modules.length];
        boolean all_modules_should_flip = true;
        for (int i = 0; i < modules.length; ++i) {
            prev_vx[i] = prevSetpoint.moduleStates[i].angle.getCos() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prev_vy[i] = prevSetpoint.moduleStates[i].angle.getSin() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prev_heading[i] = prevSetpoint.moduleStates[i].angle;
            if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = GeometryUtil.flip(prev_heading[i]);
            }
            
            desired_vx[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            desired_vy[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            desired_heading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desired_heading[i] = GeometryUtil.flip(desired_heading[i]);
            }

            if (all_modules_should_flip) {
                double required_rotation_rad = Math.abs(GeometryUtil.inverse(prev_heading[i]).rotateBy(desired_heading[i]).getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }

        if (all_modules_should_flip
                && !GeometryUtil.toTwist2d(prevSetpoint.chassisSpeeds).equals(GeometryUtil.IDENTITY) // FIXME epsilonEquals might be required here.
                && !GeometryUtil.toTwist2d(desiredState).equals(GeometryUtil.IDENTITY)) { // FIXME epsilonEquals might be required here.
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
            // of the desired
            // angle, and accelerate again.
            System.out.println("ALL MODULES FLIPPING");
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to
        // the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic
        // limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds.vyMetersPerSecond;
        double dtheta =
                desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
        // desiredState.
        double min_s = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle
        // to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
        // the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the
        // minimum across all modules, since
        // that is the active constraint.
        final double max_theta_step = dt * limits.maxSteeringVelocity;
        for (int i = 0; i < modules.length; ++i) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (epsilonEquals(prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering
                // angle, so limit based
                // purely on rotation in place.
                if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle));
                    continue;
                }

                var necessaryRotation =
                        GeometryUtil.inverse(prevSetpoint.moduleStates[i].angle)
                                .rotateBy(desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(GeometryUtil.PI);
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(
                            i,
                            Optional.of(
                                    prevSetpoint.moduleStates[i].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s =
                    findSteeringMaxS(
                            prev_vx[i],
                            prev_vy[i],
                            prev_heading[i].getRadians(),
                            desired_vx[i],
                            desired_vy[i],
                            desired_heading[i].getRadians(),
                            max_theta_step,
                            kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        // Enforce drive wheel acceleration limits.
        final double max_vel_step = dt * limits.maxDriveAcceleration;
        for (int i = 0; i < modules.length; ++i) {
            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }
            double vx_min_s =
                    min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
            double vy_min_s =
                    min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
            // already know we can't go faster than that.
            final int kMaxIterations = 250;
            double s =
                    min_s
                            * findDriveMaxS(
                                    prev_vx[i],
                                    prev_vy[i],
                                    vx_min_s,
                                    vy_min_s,
                                    max_vel_step,
                                    kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds =
                new ChassisSpeeds(
                        prevSetpoint.chassisSpeeds.vxMetersPerSecond + min_s * dx,
                        prevSetpoint.chassisSpeeds.vyMetersPerSecond + min_s * dy,
                        prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
        var retStates = _kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < modules.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(GeometryUtil.inverse(retStates[i].angle).rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation =
                    GeometryUtil.inverse(prevSetpoint.moduleStates[i].angle).rotateBy(retStates[i].angle);
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = GeometryUtil.flip(retStates[i].angle);
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        // System.out.println("min_s: "+String.valueOf(min_s));
        // System.out.println("vx: "+String.valueOf(retSpeeds.vxMetersPerSecond));
        // System.out.println("vy: "+String.valueOf(retSpeeds.vyMetersPerSecond));
        // System.out.println("omega: "+String.valueOf(retSpeeds.omegaRadiansPerSecond));
        return new SwerveSetpoint(retSpeeds, retStates);
    }
}