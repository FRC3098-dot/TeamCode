/**
 * Swerve drive subsystem.
 *
 * High-level responsibilities:
 * - Own the four physical swerve modules (frontLeft, frontRight, backLeft, backRight).
 * - Maintain and update robot odometry (pose) using module encoder positions + gyro heading.
 * - Provide convenience drive methods (field-relative vs robot-relative) that produce
 *   desired SwerveModuleState arrays and apply them to the modules.
 * - Provide utility helpers used by autonomous path-following (PathPlanner AutoBuilder).
 *
 * Important conceptual notes (read carefully when modifying):
 * - Coordinate frames and conventions:
 *   - Field-oriented (field frame): +X is forward (away from driver), +Y is left, rotation positive
 *     is counterclockwise (WPILib Rotation2d). This class converts between field and robot
 *     frames using the NavX gyro.
 *   - Robot-oriented (robot/chassis frame): +X forward, +Y left, rotation about Z positive CCW.
 * - Units:
 *   - Linear velocities are in meters and meters/second (WPILib semantics used by ChassisSpeeds
 *     and SwerveModuleState).
 *   - Angular values use radians for internal kinematics (SwerveModuleState) and degrees
 *     for human-readable telemetry. Gyro angle is read in degrees from the NavX.
 * - Odometry:
 *   - Uses WPILib's SwerveDriveOdometry. The odometry update call takes the measured
 *     module positions and the robot heading. The accuracy depends heavily on correct
 *     module encoder offsets (absolute encoders) and correct gyro zeroing.
 * - Module ordering:
 *   - Whenever module arrays are built (module states/positions), the ordering is:
 *     frontLeft, frontRight, backLeft, backRight. This ordering must match the
 *     SwerveDriveKinematics construction in DriveConstants.
 *
 * Safety, edge cases, and gotchas:
 * - Calling zeroHeading() (gyro.reset()) will alter the reference frame used for
 *   field-oriented control and odometry. Make sure to reset odometry to a known pose
 *   after zeroing the gyro if you rely on absolute field positions.
 * - Wheel speed desaturation: when converting chassis speeds to module states we
 *   desaturate wheel speeds to respect the robot's physical max speed. This preserves
 *   commanded direction/rotation proportions while clamping magnitudes.
 * - If an absolute encoder offset is incorrect or an encoder fails, odometry will
 *   drift. Keep hardware access (reading/writing motor controllers and encoders)
 *   localized here so it is easier to audit and test.
 */
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.DriveConstants;

// PathPlanner Libraries
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

@SuppressWarnings("unused")
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "Front Left");

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "Front Right");

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "Back Left");

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "Back Right");

    public boolean m_fieldOriented;
    private final Field2d field = new Field2d();
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometer =
            new SwerveDriveOdometry(
                    DriveConstants.kDriveKinematics,
                    new Rotation2d(0),
                    getModulePositions());

    public SwerveSubsystem() {

        resetAllEncoders();
        m_fieldOriented = false;

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();

        // ------------------- PathPlanner AutoBuilder Setup ------------------- //
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotRelativeSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            Constants.AutoConstants.translationConstants,
                            Constants.AutoConstants.rotationConstants),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    e.getStackTrace());
        }

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    // ------------------- Basic Robot Functions ------------------- //

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    // ------------------- Driving Logic ------------------- //

    public void drive(ChassisSpeeds speeds, boolean m_fieldOriented) {
        ChassisSpeeds chassisSpeeds = m_fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d())
                : new ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(moduleStates);
    }

    public Command setSwervePoseCmd(ChassisSpeeds lime_speeds, boolean lime_fieldoriented) {
        return runOnce(() -> drive(lime_speeds, lime_fieldoriented));
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getposition(),
                frontRight.getposition(),
                backLeft.getposition(),
                backRight.getposition()
        };
    }

    // ------------------- periodic() Method ------------------- //

    @Override
    public void periodic() {

        // Update odometry
        odometer.update(
                getRotation2d(),
                new SwerveModulePosition[]{
                        frontLeft.getposition(),
                        frontRight.getposition(),
                        backLeft.getposition(),
                        backRight.getposition()
                });

        // Update Field2d pose
        field.setRobotPose(odometer.getPoseMeters());
        SmartDashboard.putData("Field", field);

        // Update module telemetry
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location",
                getPose().getTranslation().toString());
    }

    // ------------------- Utility Functions ------------------- //

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetAllEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public double getNavxXAcceleration() {
        return gyro.getWorldLinearAccelX();
    }
}