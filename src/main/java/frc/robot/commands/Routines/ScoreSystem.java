/**
 * ScoreSystem
 *
 * <p>Purpose
 * - High-level composite command that coordinates climber/elevator actions (and optionally
 *   intake/outtake) to score a game piece. This class composes smaller commands into a
 *   logical sequence and/or parallel groups to achieve a scoring motion.
 *
 * <p>Notes
 * - This file composes lower-level commands (for example: {@code ClimberMoveCommand}) and
 *   does not directly manipulate hardware. Safety checks (limits/soft-stops) should be
 *   enforced by the underlying subsystem implementations.
 */
package frc.robot.commands.Routines;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.util.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.Climber.ClimberMoveCommand;
import frc.robot.commands.Climber.ClimberZeroHeight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Ignore unused variable warnings
@SuppressWarnings("unused") 

public class ScoreSystem extends SequentialCommandGroup{
  private final ClimberSubsystem m_climberSubsystem;
  private double m_targetHeight;
  private double outtakeTime, delayBeforeOuttake, climberTime;

  /**
   * Constructor takes a numeric target height (meters) used to move the climber.
   *
   * @param climberSubsystem climber subsystem to control (required)
   * @param targetHeight desired height in meters
   * @param delayBeforeOuttake seconds delay before outtake (unused in current impl)
   * @param outtakeTime outtake duration in seconds (unused in current impl)
   * @param climberTime duration in seconds to run the climber command
   */
  public ScoreSystem(ClimberSubsystem climberSubsystem, double targetHeight, double delayBeforeOuttake, double outtakeTime, double climberTime) {
    this.m_climberSubsystem = climberSubsystem;
    this.m_targetHeight = targetHeight;
    this.outtakeTime = outtakeTime;
    this.climberTime = climberTime;
    this.delayBeforeOuttake = delayBeforeOuttake;

    addCommands(
        new ParallelCommandGroup(
            new ClimberMoveCommand(m_climberSubsystem, m_targetHeight, climberTime)
            // new ClimberZeroHeight(m_climberSubsystem)
        )
    );
  }
}
