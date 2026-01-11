  package frc.robot;

  import com.ctre.phoenix6.controls.DutyCycleOut;

  import edu.wpi.first.math.geometry.Translation2d;
  import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.wpilibj.TimedRobot;


  public final class Constants {

    public static class ModuleConstants{
      public static final double kPhysicalMaxSpeed = 5.0;
      public static final double kTeleopMaxLinearSpeed = 5.212;
      public static final double kTeleopMaxAngularSpeed = 5.212;
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
      public static final double trackWidth = 0.6;
      public static final double wheelBase = 0.6;

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
          
      public static final int kLeftFrontModuleDriveMotorID= 22;
      public static final int kLeftFrontModuleRotationMotorID = 12;
      public static final int kLeftFrontModuleCANCoderID = 2;
      public static final double kLeftFrontModuleCANCoderOffsetDegrees = 0.010742  * 360;
      public static final boolean kLeftFrontModuleCANCoderReversed = true;
      public static final boolean kLeftFrontModuleDriveMotorReversed = false;
      public static final boolean kLeftFrontModuleDriveEncoderReversed = false;
      public static final boolean kLeftFrontModuleRotationMotorReversed = false;
      public static final boolean kLeftFrontModuleRotationEncoderReversed = false;
      
      public static final int kLeftRearModuleDriveMotorID= 11;
      public static final int kLeftRearModuleRotationMotorID = 10;
      public static final int kLeftRearModuleCANCoderID = 1;
      public static final double kLeftRearModuleCANCoderOffsetDegrees = 0.003662     * 360;
      public static final boolean kLeftRearModuleCANCoderReversed = true;
      public static final boolean kLeftRearModuleDriveMotorReversed = false;
      public static final boolean kLeftRearModuleDriveEncoderReversed = false;
      public static final boolean kLeftRearModuleRotationMotorReversed = false;
      public static final boolean kLeftRearModuleRotationEncoderReversed = false;
        
      public static final int kRightFrontModuleDriveMotorID= 33;
      public static final int kRightFrontModuleRotationMotorID = 13;
      public static final int kRightFrontModuleCANCoderID = 3;
      public static final double kRightFrontModuleCANCoderOffsetDegrees = -0.012207   * 360;
      public static final boolean kRightFrontModuleCANCoderReversed = true;
      public static final boolean kRightFrontModuleDriveMotorReversed = true;
      public static final boolean kRightFrontModuleDriveEncoderReversed = false;
      public static final boolean kRightFrontModuleRotationMotorReversed = false;
      public static final boolean kRightFrontModuleRotationEncoderReversed = false;

      public static final int kRightRearModuleDriveMotorID= 44;
      public static final int kRightRearModuleRotationMotorID = 14;
      public static final int kRightRearModuleCANCoderID = 4;
      public static final double kRightRearModuleCANCoderOffsetDegrees = 0.027588  * 360;
      public static final boolean kRightRearModuleCANCoderReversed = true;
      public static final boolean kRightRearModuleDriveMotorReversed = true ;
      public static final boolean kRightRearModuleDriveEncoderReversed = false;
      public static final boolean kRightRearModuleRotationMotorReversed = false;
      public static final boolean kRightRearModuleRotationEncoderReversed = false;
      

    }
  }
