  package frc.robot;
  import com.ctre.phoenix6.controls.DutyCycleOut;
  import com.pathplanner.lib.config.PIDConstants;
  import edu.wpi.first.math.geometry.Translation2d;
  import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.wpilibj.TimedRobot;
  public final class Constants {

     public static class ModuleConstants{
      public static final double kPhysicalMaxSpeed = 5.0;

         public static final double kAutoMaxLinearSpeed = 2.; // m/s
    public static final double kAutoMaxAngularSpeed = Math.PI; // rad/s


      public static final double kTeleopMaxLinearSpeed =  4.2;
      public static final double kTeleopMaxAngularSpeed = 3.8;
      
      public static final double kSteeringGearRatio = 21.4287/1;
      public static final double kDriveGearRatio = 6.12/1;
      public static final double kRotationP  = 0.009;
      public static final double kWheelDiameter = 0.093;
      public static final double kWheelCircumference = 0.093 * Math.PI;
      public static final double kDriveEncoderPositionConversionFactor = kWheelCircumference/kDriveGearRatio;
      public static final double kDriveEncoderVelocityConversionFactor = kDriveEncoderPositionConversionFactor/60;
      public static final double kRotationEncoderPositionConversionFactor = 360/kSteeringGearRatio;
      public static final double kRotationEncoderVelocityConversionFactor = kRotationEncoderPositionConversionFactor/60;
      public static DutyCycleOut velocity = new DutyCycleOut(kDriveEncoderVelocityConversionFactor);
      public static final double kPeriod = TimedRobot.kDefaultPeriod;
    }
    public static class SwerveSubsystemConstants{
      public static final double trackWidth = 0.55; //0.6
      public static final double wheelBase = 0.55;   //0.6

      public static final double kSlewRateXLimit = 500;
      public static final double kSlewRateYLimit = 500;
      public static final double kSlewRateOmegaLimit = 500;
      
      //left front , left rear, right front, right rear
      public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase/2, -trackWidth/2 ), //solön
        new Translation2d(-wheelBase/2, -trackWidth/2), //solarka
        new Translation2d(wheelBase/2, trackWidth/2 ),   //sağön
        new Translation2d(-wheelBase/2, trackWidth/2)  //sağarka
      );

      public static final int pigeonID =50;
      //oldu
      public static final int kLeftRearModuleDriveMotorID= 5;
      public static final int kLeftRearModuleRotationMotorID = 3;
      public static final int kLeftRearModuleCANCoderID = 33;
      public static final double kLeftRearModuleCANCoderOffsetDegrees = 0.736328 * 360;
      public static final boolean kLeftRearModuleCANCoderReversed = true; //
      public static final boolean kLeftRearModuleDriveMotorReversed = false;
      public static final boolean kLeftRearModuleDriveEncoderReversed = false;
      public static final boolean kLeftRearModuleRotationMotorReversed = false;
      public static final boolean kLeftRearModuleRotationEncoderReversed = false;
      //oldu
      public static final int kLeftFrontModuleDriveMotorID= 8;
      public static final int kLeftFrontModuleRotationMotorID = 4;
      public static final int kLeftFrontModuleCANCoderID = 40;
      public static final double kLeftFrontModuleCANCoderOffsetDegrees = 0.058105 * 360;
      public static final boolean kLeftFrontModuleCANCoderReversed = true; //
      public static final boolean kLeftFrontModuleDriveMotorReversed = false;
      public static final boolean kLeftFrontModuleDriveEncoderReversed = false;
      public static final boolean kLeftFrontModuleRotationMotorReversed = false;
      public static final boolean kLeftFrontModuleRotationEncoderReversed = false;
        //oldu
      public static final int kRightFrontModuleDriveMotorID= 1;
      public static final int kRightFrontModuleRotationMotorID = 2;
      public static final int kRightFrontModuleCANCoderID = 10;
      public static final double kRightFrontModuleCANCoderOffsetDegrees = 0.214355 * 360;
      public static final boolean kRightFrontModuleCANCoderReversed = true; //
      public static final boolean kRightFrontModuleDriveMotorReversed = true;
      public static final boolean kRightFrontModuleDriveEncoderReversed = false;
      public static final boolean kRightFrontModuleRotationMotorReversed = false;
      public static final boolean kRightFrontModuleRotationEncoderReversed = false;

      public static final int kRightRearModuleDriveMotorID= 6;
      public static final int kRightRearModuleRotationMotorID = 7;
      public static final int kRightRearModuleCANCoderID = 11;
      public static final double kRightRearModuleCANCoderOffsetDegrees = 0.752930 * 360;
      public static final boolean kRightRearModuleCANCoderReversed = true; //
      public static final boolean kRightRearModuleDriveMotorReversed = true ;
      public static final boolean kRightRearModuleDriveEncoderReversed = false;
      public static final boolean kRightRearModuleRotationMotorReversed = false;
      public static final boolean kRightRearModuleRotationEncoderReversed = false;
      
      public static final PIDConstants translationConstants = 
      new PIDConstants(3.0, 0.0, 0);  
      public static final PIDConstants rotationConstants = 
      new PIDConstants(3, 0, 0);  
    }
  }
