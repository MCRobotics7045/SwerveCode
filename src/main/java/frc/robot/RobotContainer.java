// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignWithAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.ControlandCommand.*;
import static frc.robot.Constants.Vision.*;


public class RobotContainer {
  private final VisionSubsystem vision = new VisionSubsystem(); 
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;
  // private final Pigeon2 IMU_Pigeon = new Pigeon2(Constants.Pigeon2IMUid);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  private final Telemetry logger = new Telemetry(MaxSpeed);
  

  private final XboxController XBOX = new XboxController(XBOX_CONTROLLER_PORT);

  public double JoyX = XBOX.getLeftX();
  public double JoyY = XBOX.getLeftY();

  
  public RobotContainer() {
    System.out.println("Robot Started ");
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    



    
  }


  private void configureBindings() {
    System.out.println("Config Bind Ready");
    
    //=================================================================================================================================
    // final POVButton dPadRight = new POVButton(XBOX, 90);
    // final POVButton dPadLeft = new POVButton(XBOX, 270);
    // final POVButton dPadUp = new POVButton(XBOX, 0);
    // final POVButton dPadDown = new POVButton(XBOX, 180);
    // final JoystickButton buttonY = new JoystickButton(XBOX, Constants.xboxYellowButton);
    final JoystickButton buttonA = new JoystickButton(XBOX, xboxGreenButton);   
    final JoystickButton buttonX = new JoystickButton(XBOX, xboxBlueButton);
    final JoystickButton buttonB = new JoystickButton(XBOX, xboxRedButton);
    // final JoystickButton buttonRB = new JoystickButton(XBOX, Constants.xboxRBButton);
    final JoystickButton buttonLB = new JoystickButton(XBOX, xboxLBButton);
    // final JoystickButton buttonStart = new JoystickButton(XBOX, Constants.xboxStartButton);
    // final JoystickButton buttonMENU = new JoystickButton(XBOX, Constants.xboxMenuButton);
   




    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive 
        
    //     .withVelocityX(-XBOX.getLeftX() * MaxSpeed) // Apply deadzone on X-axis
    //     .withVelocityY(-XBOX.getLeftY() * MaxSpeed) // Apply deadzone on Y-axis
    //     .withRotationalRate(-XBOX.getRightX()* MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));






    

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive 
        
        .withVelocityX(-applyDeadzone(XBOX.getLeftX(), xboxDeadzoneStickLeft_X) * MaxSpeed) // Apply deadzone on X-axis
        .withVelocityY(-applyDeadzone(XBOX.getLeftY(), xboxDeadzoneStickLeft_Y) * MaxSpeed) // Apply deadzone on Y-axis
        .withRotationalRate(-applyDeadzone(XBOX.getRightX(), xboxDeadzoneStickRight_X) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    buttonX.onTrue(new AlignWithAprilTag(drivetrain, Apriltag14, drive));
    buttonA.whileTrue(drivetrain.applyRequest(() -> brake));
    buttonB.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    buttonLB.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //Deadzone command that takes XBOX value and matches the correct sign of +/_ and then subtracts it form the deadzone than divides to even it out
  private double applyDeadzone(double stickvalue, double nonozone) {
    if (Math.abs(stickvalue) < nonozone) {
        return 0.0;
    }
    return (stickvalue - Math.copySign(nonozone, stickvalue)) / (1.0 - nonozone);
}

  public CommandSwerveDrivetrain getSwerveDrivetrain() {
    return drivetrain;
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
