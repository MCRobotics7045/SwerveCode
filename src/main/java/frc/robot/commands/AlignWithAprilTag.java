// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.RobotContainer;

public class AlignWithAprilTag extends Command {
  /** Creates a new AlignWithAprilTag. */
  private RobotContainer container;
  private container.drive drive;
  private VisionSubsystem Vision;
  private CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
  private double forward;
  private double strafe;
  private double turn;
  private int TagID;
  private final double VISION_TURN_kP = 0.01;;
 
  public AlignWithAprilTag(CommandSwerveDrivetrain m_swerve , double m_forward, double m_strafe, double m_turn,int m_TagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;
    forward = m_forward;
    strafe = m_strafe;
    turn = m_turn;
    TagID = m_TagID;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = -1.0 * Vision.FindTagIDyaw(TagID) * VISION_TURN_kP * Constants.kMaxAngularSpeed;
    
    swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive 
        
        .withVelocityX(-applyDeadzone(XBOX.getLeftX(), Constants.xboxDeadzoneStickLeft_X) * MaxSpeed) // Apply deadzone on X-axis
        .withVelocityY(-applyDeadzone(XBOX.getLeftY(), Constants.xboxDeadzoneStickLeft_Y) * MaxSpeed) // Apply deadzone on Y-axis
        .withRotationalRate(-applyDeadzone(XBOX.getRightX(), Constants.xboxDeadzoneStickRight_X) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
