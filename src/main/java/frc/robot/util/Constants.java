package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Application-wide constants for the robot project.
 *
 * <p>This class is intended as the single authoritative place for tuning numbers, sensor offsets,
 * CAN IDs, port mappings, unit conversion factors, and any other global constants. Keep the
 * constants grouped into meaningful nested static classes (for example: DriveConstants,
 * ElevatorConstants). Avoid putting logic here; this file should contain only immutable values
 * (or tunable static values) and small enums describing fixed choices.
 *
 * <p>Guidelines:
 * - Units: prefer meters (m), radians (rad), seconds (s) for physical values. Where values are
 *   expressed in other units (inches, degrees), include a comment and convert with
 *   {@link edu.wpi.first.math.util.Units}.
 * - Naming: prefix constants with the subsystem name (kDrive..., kElevator...) to reduce
 *   accidental collisions and make usage self-documenting.
 * - Offsets: store sensor offsets (for example encoder zero offsets) as radians/meters and
 *   document the coordinate frame the offset refers to.
 *
 * <p>When changing an existing constant, add a short note explaining the reason and the source
 * (measured value, datasheet, or derived calculation). This helps future debugging and tuning.
 */
public final class Constants {

    /**
     * Constants that describe the per-module (individual swerve module) geometry and encoder
     * conversion factors.
     *
     * <p>Conversions below follow the pattern:
     * <ul>
     *   <li>distance per motor revolution = gearRatio * wheelCircumference (meters)</li>
     *   <li>angle per motor revolution = gearRatio * 2 * PI (radians)</li>
     * </ul>
     *
     * Notes:
     * - Gear ratio is expressed as motorRotations / outputRotations (so smaller numbers mean
     *   more motor rotations per output rotation). Verify that the ratios match your motor
     *   controller configuration (some libraries expect the inverse).
     */
    public static final class ModuleConstants {
        /** Wheel diameter used for linear distance conversions (meters). */
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);

        /** Ratio from motor rotations to wheel rotations (motorRotations / wheelRotations). */
        public static final double kDriveMotorGearRatio = 1 / 5.08;

        /** Ratio from motor rotations to steering (turning) rotations. */
        public static final double kTurningMotorGearRatio = 1 / 46.42;

        /**
         * Converts a drive-motor rotation to meters traveled by the wheel.
         * Calculation: motorRot * gearRatio * circumference
         */
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

        /** Converts a turning-motor rotation to radians of steering rotation. */
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        /** RPM -> meters / second for drive encoder. */
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

        /** RPM -> radians / second for turning encoder. */
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        /** Proportional gain used by the turning PID loop. Tune if modules oscillate or are slow. */
        public static final double kPTurning = 0.15;
    }

    /**
     * Drive subsystem physical and tuning constants.
     *
     * <p>This class contains CAN IDs for the drive and turning motors, physical dimensions used
     * to construct the kinematics object, encoder polarity flags, maximum speeds used for
     * trajectory generation / teleop limits, and controller deadbands. Keep motor port numbers
     * consistent with the robot wiring and roboRIO assignment.
     */
    public static final class DriveConstants {
        /** Distance between left and right wheels (meters). Used by kinematics. */
        public static final double kTrackWidth = Units.inchesToMeters(24);

        /** Distance between front and back wheels (meters). Used by kinematics. */
        public static final double kWheelBase = Units.inchesToMeters(24);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

       /** This is the Raw angle devided 360 */
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.129;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.177;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.983;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  0.981;


    /** Theoretical maximum linear speed of the robot (m/s). Use conservatively. */
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    /** Theoretical maximum angular speed (rad/s). 4 * PI corresponds to 2 full rotations/sec. */
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /** Deadband applied to joystick translation axis (unitless, 0-1). Helps ignore small stick noise. */
        public static double kControllerDeadband = 0.25;

        /** Deadband applied to joystick rotation axis (unitless, 0-1). */
        public static double kControllerRotDeadband = 0.15;
    }

    /**
     * Autonomous driving and path-following tuning constants. These values are used by the
     * trajectory follower and nested controllers (PID / feedforward). Reduce them conservatively
     * when testing on the real robot.
     */
    public static final class AutoConstants {
        /** Max translation speed used while generating autonomous trajectories (m/s). */
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 20.0;
        public static final double kPYController = 18.0;
        public static final double kPThetaController = 3.0;
        public static final double kIThetaController = 0.2;
        public static final double kDThetaController = 0.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

        /** PID constants for translation controller (PathPlanner / Trajectory following). */
        public static final PIDConstants translationConstants = new PIDConstants(17.0, 0.05, 0.0);

        /** PID constants for rotation controller used during path following. */
        public static final PIDConstants rotationConstants = new PIDConstants(10.0, 0.05, 0.0);
    }

    /**
     * Operator Interface constants: controller ports, axis indices and button mappings.
     *
     * <p>Button and axis numbering follows the controller library used (typically WPILib/Xbox
     * mapping). If you swap controllers, verify these indices match the new device.
     */
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 5; // Lt Bumper
        public static final int kdriverBack = 7;

        public static final int kCodriver_B = 2;
        public static final int kCodriver_Y = 4;
        public static final int kCodriver_A = 1;
        public static final int kCpdriver_X = 3;

        public static final int kCodriverRTrigger = 3;
        public static final int kCodriverLTrigger = 2;
        public static final int kCodriverRBumper = 6;
        public static final int kCodriverLBumper = 5;
        public static final int kCodriverBack = 7;

        public static final double kCodriverLJoystickY = 1;
        public static final double kCodriverRJoystickY = 5;
        public static final double kDeadband = 0.05;
    }

    /** PWM and addressable LED port assignments. */
    public static final class PWMPorts {
        /** PWM port for the Blinkin LED controller (on PDP or roboRIO PWM header). */
        public static final int BLINKIN_LED_CONTROLLER_PORT = 0;

        /** Port used for addressable LED output (if present). */
        public static final int kAddressableLED = 9;
    }

    /**
     * Elevator subsystem constants (gearing, limits, PID, feedforward).
     *
     * <p>Key fields:
     * - kElevatorMinHtMeters / kElevatorMaxHtMeters: safe travel limits (meters). Ensure
     *   limit switches or soft limits enforce these in the subsystem.
     * - Feedforward (kS, kG, kV, kA): used with WPILib's ElevatorFeedforward or similar.
     */
    public static final class ClimberConstants {
    public static final int kElevatorMotor = 10;
        public static final int kMinHeightLimit = 5;
        public static final double kElDeadband = 0.05;
        public static final double kElMotorGain = 0.5;
        public static final double kElMotorSlew = 3;
    public static final double kElevatorTime = 5.0;

        public static final double kElevGearDiameterMeters = 0.0480822;
        public static final double kGearDIAMeters = Units.inchesToMeters(1.9);
        public static final double kgearJunk = kGearDIAMeters * 1.0;
        public static final double kElevGearRatio = 1 / 10.0;
        public static final double kElevRot2Meters = kElevGearRatio * Math.PI * kElevGearDiameterMeters;
        public static final double kElevatorRotRPMRadsperSecond = kElevRot2Meters / 60;
        public static final double kElevatorMinHtMeters = Units.inchesToMeters(1);
        public static final double kElevatorMaxHtMeters = Units.inchesToMeters(28.5);

        public static final double kElevatorkP = 4.5;
        public static final double kElevatorkI = 0.0;
        public static final double kElevatorkD = 0.0;
        public static final double kElevatorkFF = 0.0;
        public static final double kElevatorkIz = 0;

    // Feedforward terms (S, G, V, A) used by ElevatorFeedforward. Units: V (volts), m/s, m/s^2.
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;

        public static final double kElevatorRampRate = 0.1;

        /** Constraints used for motion-profiled elevator movement (velocity in m/s). */
        public static final class TrapezoidProfileConstants {
            public static final double kElevatorMaxVelocity = 3.67;
            public static final double kElevatorMaxAcceleration = 3.0;
            public static final TrapezoidProfile.Constraints kElevatorCONSTRAINTS = new TrapezoidProfile.Constraints(
                kElevatorMaxVelocity, kElevatorMaxAcceleration);
        }
    }

    // Coral intake removed: constants and subsystem were deleted to simplify robot configuration.

    /** Conveyor subsystem constants: motor ID and default run speed. */
    public static final class ConveyorConstants {
        /** CAN ID for the conveyor motor. Update to match robot wiring. */
        public static final int kConveyorMotor = 15;

        /** Default conveyor speed (unitless, -1 to 1). Positive should move cargo toward shooter. */
    public static final double kConveyorSpeed = 0.5;
    }

    /** IntakeExtand subsystem constants: motor ID and default run speed. */
    public static final class IntakeExtandConstants {
        /** CAN ID for the intake-extand motor. Update to match robot wiring. */
        public static final int kIntakeExtandMotor = 21;

        /** Default intake-extand speed (unitless, -1 to 1). Positive should move cargo toward shooter. */
        public static final double kIntakeExtandSpeed = 0.6; // 60%
    }

    /** Shooter subsystem constants: flywheels and indexer motor IDs and speeds. */
    public static final class ShooterConstants {
        // CAN IDs - update to match your robot wiring
    public static final int kFlywheelBack = 16;
    public static final int kFlywheelFront = 17;
    // Name the single/star indexer consistently with the other indexers (Front/Back)
    public static final int kIndexerStar = 18;
    public static final int kIndexerFront = 19;
    public static final int kIndexerBack = 20;
        // Default speeds
        public static final double kFlywheelSpeed = 0.6; // 60%
        public static final double kIndexerSpeed = 0.6; // 60%
        public static final double kIndexerOnlyStarSpeed = 0.6; // 60%

        // Axis threshold for considering trigger "pressed"
        public static final double kTriggerThreshold = 0.1;
    // Map the Top/Bottom / OnlyStar aliases to the explicit front/back/star CAN IDs
    // This avoids uninitialized defaults (0) when subsystems construct motor objects.
   /* public static int kFlywheelTop = kFlywheelBack;
    public static int kFlywheelBottom = kFlywheelFront;
    public static int kIndexerTop = kIndexerBack;
    public static int kIndexerBottom = kIndexerFront; */
    }

    // ---------------- Fuel / Arm constants ----------------       
    /**
     * Fuel/arm subsystem constants. Includes CAN IDs, limit switches, gear geometry, and PID
     * / feedforward values for the arm mechanism.
     */
    public static final class FuelConstants {
        // CAN IDs
        public static final int kFuelIntake = 13;
        // DIO Port
        public static final int kFuelIntakeLimit = 6;

    public static final double kFuelLoadGain = 0.5;
    public static final double kFuelUnloadGain = -0.5;
        public static final double kFuelDeadband = 0.05;

    // Arm (previously named Fuel) constants used by ArmSubsystem
    // NOTE: identifiers previously containing the word 'Fuel' have been renamed to use
    // the term 'Fuel' to match current subsystem naming.
    public static final int kFuelArm = 14; // motor CAN ID for arm (matches kArm)
        public static final double kArmGearDiameterMeters = Units.inchesToMeters(2.7);
        public static final double kArmGearRatio = 1 / 40.0;
        public static final double kArmRot2Rad = kArmGearRatio * 2 * Math.PI;
        public static final double kArmRotRPMRadsperSecond = kArmRot2Rad / 60.0;
        public static final double kArmPosition = Units.degreesToRadians(50.0);
        public static final double kArmRampRate = 0.5;

    /** Motion profile constraints for the arm (rad/s and rad/s^2 or units used by your profile). */
    public static final class TrapezoidProfileConstants {
        public static final double kFuelArmMaxVelocity = 3.0;
        public static final double kFuelArmMaxAcceleration = 1.5;
        public static final TrapezoidProfile.Constraints kFuelArmCONSTRAINTS = new TrapezoidProfile.Constraints(
            kFuelArmMaxVelocity, kFuelArmMaxAcceleration);
    }

    public static final double kFuelArmkP = 0.025;
    public static final double kFuelArmkI = 0.0;
    public static final double kFuelArmkD = 0.0;
    public static final double kFuelArmkFF = 0.0;
    public static final double kFuelArmkIz = 0;

    public static final double kFuelArmkG = 0.0535;
    public static final double kFuelArmkV = 0.0175;
    public static final double kFuelArmkS = 0.0;
    public static final double kFuelArmkA = 0.0;
    }

    // Reef-related presets removed.

    /** POV (D-pad) angle mappings used by driver input code. These are the standard angles
     * returned by controller libraries when reading the D-pad. */
    public static final class POV {
        public static final int UP = 0;
        public static final int UP_RIGHT = 45;
        public static final int RIGHT = 90;
        public static final int DOWN_RIGHT = 135;
        public static final int DOWN = 180;
        public static final int DOWN_LEFT = 225;
        public static final int LEFT = 270;
        public static final int UP_LEFT = 315;
    }

    /**
     * Vision system constants and known AprilTag positions.
     *
     * <p>Camera heights and mount angles are used to convert Limelight pitch angles into
     * distance estimates. Verify these values in the field and update the <code>
     * the camera or mount changes.
     */
    public static final class VisionConstants {
    /** Primary (front) camera name used by most subsystems. */
    public static final String CameraFront = "limelight-frtcam";

    /** Secondary (back) camera name. */
    public static final String CameraBack = "limelight-bckcam";

    /** Optional third/top camera name. Update to match your network table names. */
    public static final String CameraTop = "limelight-topcam";

    // Backwards-compatible alias used by older code
    public static final String CameraName = CameraFront; // default camera name
        // TODO: Update values for Robot Limelight after physical verification

        /** Camera height above the floor (meters). */
        public static final double hCamera = 0.6; // camera height in meters

        /** Target (goal) height above the floor (meters). */
        public static final double hTarget = 2.0; // target height in metersmake 

        /** Camera mounting pitch angle (degrees). Positive values indicate the camera is pitched up. */
        public static final double cameraMountAngle = 30.0; // degrees TODO: due to change

        public static enum AprilTagPose {
            TAG_1(1, 0.0, 0.0, 0.0),
            TAG_2(2, 0.0, 0.0, 0.0),
            TAG_3(3, 0.0, 0.0, 0.0),
            TAG_4(4, 0.0, 0.0, 0.0),
            TAG_5(5, 0.0, 0.0, 0.0),
            TAG_6(6, 0.0, 0.0, 0.0),
            TAG_7(7, 0.0, 0.0, 0.0),
            TAG_8(8, 0.0, 0.0, 0.0),
            TAG_9(9, 0.0, 0.0, 0.0),
            TAG_10(10, 0.0, 0.0, 0.0),
            TAG_11(11, 0.0, 0.0, 0.0),
            TAG_12(12, 0.0, 0.0, 0.0),
            TAG_13(13, 0.0, 0.0, 0.0),
            TAG_14(14, 0.0, 0.0, 0.0),
            TAG_15(15, 0.0, 0.0, 0.0),
            TAG_16(16, 0.0, 0.0, 0.0),
            TAG_17(17, 0.0, 0.0, 0.0),
            TAG_18(18, 0.0, 0.0, 0.0),
            TAG_19(19, 0.0, 0.0, 0.0),
            TAG_20(20, 0.0, 0.0, 0.0),
            TAG_21(21, 0.0, 0.0, 0.0),
            TAG_22(22, 0.0, 0.0, 0.0),
            TAG_23(23, 0.0, 0.0, 0.0),
            TAG_24(24, 0.0, 0.0, 0.0),
            TAG_25(25, 0.0, 0.0, 0.0),
            TAG_26(26, 0.0, 0.0, 0.0),
            TAG_27(27, 0.0, 0.0, 0.0),
            TAG_28(28, 0.0, 0.0, 0.0),
            TAG_29(29, 0.0, 0.0, 0.0),
            TAG_30(30, 0.0, 0.0, 0.0),
            TAG_31(31, 0.0, 0.0, 0.0),
            TAG_32(32, 0.0, 0.0, 0.0);
            

            private final int tagID;
            private final double xPose;
            private final double yPose;
            private final double angle;

            private AprilTagPose(int tagID, double xPose, double yPose, double angle) {
                this.tagID = tagID;
                this.xPose = xPose;
                this.yPose = yPose;
                this.angle = angle;
            }

            public int getTagID() {
                return tagID;
            }

            public double getXPosition() {
                return xPose;
            }

            public double getYPosition() {
                return yPose;
            }

            public double getAngle() {
                return angle;
            }
        }
    }
}
