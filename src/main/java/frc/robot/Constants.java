/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean kDebug = true;



    // XBOX Buttons
    public static final int xboxGreenButton = 1;
    public static final int xboxRedButton = 2;
    public static final int xboxBlueButton = 3;
    public static final int xboxYellowButton = 4;
    public static final int xboxLBButton = 5;
    public static final int xboxRBButton = 6;
    public static final int xboxStartButton = 7;
    public static final int xboxMenuButton = 8;
    public static final int xboxLStickButton = 9;
    public static final int xboxRStickButton = 10;

    public static final double xboxStickDeadband = 0.05;  //Joysticks need to exceed this value

    // USB Ports
    public static final int XBOX_CONTROLLER_PORT = 0;

   //Deadzones

   public static final double xboxDeadzoneStickLeft_X = 0.05; //very sorry for long name i just need to get it correct
   public static final double xboxDeadzoneStickLeft_Y = 0.075;
   public static final double xboxDeadzoneStickRight_X = 0.1; //this depends on controller 
   public static final double xboxDeadzoneStickRight_Y = 0.1;



   public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
}
