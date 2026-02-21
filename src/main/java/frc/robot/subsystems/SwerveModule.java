/**
 * Individual swerve module implementation.
 *
 * <p>This class encapsulates the drive and turning motors/encoders for a single swerve module
 * and provides conversion utilities between motor rotations and physical units. Keep module-level
 * control (angle optimization, state conversion) inside this class.
 */
package frc.robot.subsystems;

//REV Controller Libraries
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.SparkPIDController;
//import com.revrobotics.SparkBase.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


// Math Functions
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//Commands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;

//Robot Commands, Subsystems, Constants
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ModuleConstants;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig;
   
    private final SparkMax turningMotor;
    private final SparkMaxConfig turnMotorConfig;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final DutyCycleEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private String moduleName;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        absoluteEncoder.setDutyCycleRange(1/4096, 4095);
        //absoluteEncoder.setDistancePerRotation(2*Math.PI); Depricated
        moduleName=name;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotorConfig= new SparkMaxConfig();
        turnMotorConfig= new SparkMaxConfig();
        
        
        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveMotorConfig.inverted(driveMotorReversed);
        driveMotorConfig.smartCurrentLimit(40);
       
       
        turnMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turnMotorConfig.inverted(turningMotorReversed);
        turnMotorConfig.smartCurrentLimit(40);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

       
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    // Call resetEncoders
        resetEncoders();
    }
    public void update(){
       
         SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());
         SmartDashboard.putNumber(moduleName + "Absolute-Position Corrected", getAbsoluteEncoderRad());
         SmartDashboard.putNumber(moduleName + "Encoder Position", absoluteEncoder.get());
         SmartDashboard.putNumber(moduleName + "Drive Position", getDrivePosition());
         SmartDashboard.putNumber(moduleName + "Drive Velocity", getDriveVelocity());
       }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    public SwerveModulePosition getposition() { 
        return (new SwerveModulePosition(getDrivePosition(),(new Rotation2d(getTurningPosition()))));
    }

    public double getAbsoluteEncoderRad() {
        double angle = Math.abs(absoluteEncoder.get());
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad*2*Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
   
  /**
  * Minimize the change in heading the desired swerve module state would require by potentially
  * reversing the direction the wheel spins. If this is used with the PIDController class's
  * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
  *
  * @param desiredState The desired state.
  * @param currentAngle The current module angle.
  * @return Optimized swerve module state.
  */

   public static SwerveModuleState optimize(
         SwerveModuleState desiredState, Rotation2d currentAngle) {
       var delta = desiredState.angle.minus(currentAngle);
       if (Math.abs(delta.getDegrees()) > 90.0) {
          return new SwerveModuleState(
             -desiredState.speedMetersPerSecond,
              desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
       } else {
         return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
       }
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = optimize(state, getState().angle); 
        //state = SwerveModuleState.optimize(state, getState().angle); //Depricated
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
       // SmartDashboard.putString("Swerve[" + absoluteEncoder.get() + "] state", state.toString());
     
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}