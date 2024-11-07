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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;
import frc.robot.Simulation.SimulationTele;
import frc.robot.commands.AlignWithAprilTag;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.DriveCommands.DefaultDrive;

import static frc.robot.Constants.Constants.InputConstants.*;

import org.photonvision.PhotonCamera;


public class RobotContainer {

   private final XboxController XBOX = new XboxController(XBOX_CONTROLLER_PORT);

 
  private final SwerveSubsystem SWERVE = TunerConstants.DriveTrain;
  private final VisionSubsystem VISION  = new VisionSubsystem(); 

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    
    System.out.println("Robot Started ");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));

    SWERVE.setDefaultCommand(new DefaultDrive(XBOX,SWERVE));
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
    
    
  
  }


   public static boolean IsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

  
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
