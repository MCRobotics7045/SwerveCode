// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Phoniex https://api.ctr-electronics.com/phoenix6/release/java/

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSub;
import frc.robot.commands.AlignWithAprilTag;
// import frc.robot.commands.AlignWithAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.ControlandCommand.*;
import frc.robot.commands.FireCommand;
import org.photonvision.PhotonCamera;


public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  

  //-----------------------------------------Swerve Defnintions--------------------------
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  //--subsystems--
  private final VisionSubsystem vision  = new VisionSubsystem(); 
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //controllers
  private final XboxController XBOX = new XboxController(XBOX_CONTROLLER_PORT);
  PhotonCamera piCamera1 = new PhotonCamera("Pi_Camera");

  private final IntakeSub intake = new IntakeSub();

  public RobotContainer() {
    
    System.out.println("Robot Started ");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));
    configureBindings();
  }


  private void configureBindings() {
    System.out.println("Config Bind Ready");
    //=================================================================================================================================
    // final POVButton dPadRight = new POVButton(XBOX, 90);
    // final POVButton dPadLeft = new POVButton(XBOX, 270);
    // final POVButton dPadUp = new POVButton(XBOX, 0);
    // final POVButton dPadDown = new POVButton(XBOX, 180);
    final JoystickButton buttonY = new JoystickButton(XBOX, xboxYellowButton);
    final JoystickButton buttonA = new JoystickButton(XBOX, xboxGreenButton);   
    final JoystickButton buttonX = new JoystickButton(XBOX, xboxBlueButton);
    final JoystickButton buttonB = new JoystickButton(XBOX, xboxRedButton);
    // final JoystickButton buttonRB = new JoystickButton(XBOX, xboxRBButton);
    final JoystickButton buttonLB = new JoystickButton(XBOX, xboxLBButton);
    // final JoystickButton buttonStart = new JoystickButton(XBOX, xboxStartButton);
    // final JoystickButton buttonMENU = new JoystickButton(XBOX, xboxMenuButton);
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive 
        
        .withVelocityX(-applyDeadzone(XBOX.getLeftX(), xboxDeadzoneStickLeft_X) * MaxSpeed) // Apply deadzone on X-axis
        .withVelocityY(-applyDeadzone(XBOX.getLeftY(), xboxDeadzoneStickLeft_Y) * MaxSpeed) // Apply deadzone on Y-axis
        .withRotationalRate(-applyDeadzone(XBOX.getRightX(), xboxDeadzoneStickRight_X) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    buttonX.onTrue(new FireCommand(intake, vision));
    buttonY.whileTrue(new AlignWithAprilTag(drivetrain, drive,piCamera1));
    buttonA.whileTrue(drivetrain.applyRequest(() -> brake));
    buttonB.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-XBOX.getLeftY(), -XBOX.getLeftX()))));
    buttonLB.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  
  //Deadzone commnd that takes XBOX value and matches the correct sign of +/_ and then subtracts it form the deadzone than divides to even it out
  private double applyDeadzone(double stickvalue, double nonozone) {
    if (Math.abs(stickvalue) < nonozone) {
        return 0.0;
    }
    return (stickvalue - Math.copySign(nonozone, stickvalue)) / (1.0 - nonozone);
  }



  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
