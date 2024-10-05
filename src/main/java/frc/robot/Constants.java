/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    //Can IDS
    public static final int Pigeon2IMUid = 0;

    // XBOX Buttons
    


    //Align With April Tag Command
    


   
    
    public static class ControlandCommand {

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
    }


    public static class Vision {

        public static AprilTagFieldLayout kTagLayout = null;
                
         public static AprilTagFieldLayout getTagLayout() {
            if (kTagLayout == null) {
                kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            }
                return kTagLayout;
            }
        
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double ErrorMarginPostive = 2.0;
        public static final double ErrorMarginNegative = -2.0;
        public static final double VISION_TURN_kP = 0.5;

         //AprilTag Id THESE WILL BE NAMED TO THERE SPOTS ON FEILD FOR 2025
        public static final int Apriltag1 = 1;
        public static final int Apriltag2 = 2;
        public static final int Apriltag3 = 3;
        public static final int Apriltag4 = 4;
        public static final int Apriltag5 = 5;
        public static final int Apriltag6 = 6;
        public static final int Apriltag7 = 7;
        public static final int Apriltag8 = 8;
        public static final int Apriltag9 = 9;
        public static final int Apriltag10 = 10;
        public static final int Apriltag11 = 11;
        public static final int Apriltag12 = 12;
        public static final int Apriltag13 = 13;
        public static final int Apriltag14 = 14;
        public static final int Apriltag15 = 15;
        public static final int Apriltag16 = 16;
        
        public static int BestFoundTag = 0;
    }

}
