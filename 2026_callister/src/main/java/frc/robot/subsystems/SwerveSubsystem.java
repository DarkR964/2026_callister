package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Joule;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveSubsystem extends SubsystemBase {
  CANBusStatus can = new CANBusStatus();
  

  SwerveModule leftFrontModule = new SwerveModule(
  Constants.SwerveSubsystemConstants.kLeftFrontModuleDriveMotorID,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleRotationMotorID,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleCANCoderID,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleCANCoderOffsetDegrees,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleDriveMotorReversed,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleRotationMotorReversed,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleDriveEncoderReversed,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleRotationEncoderReversed,
  Constants.SwerveSubsystemConstants.kLeftFrontModuleCANCoderReversed);

  SwerveModule leftRearModule = new SwerveModule(
  Constants.SwerveSubsystemConstants.kLeftRearModuleDriveMotorID,
  Constants.SwerveSubsystemConstants.kLeftRearModuleRotationMotorID,
  Constants.SwerveSubsystemConstants.kLeftRearModuleCANCoderID,
  Constants.SwerveSubsystemConstants.kLeftRearModuleCANCoderOffsetDegrees,
  Constants.SwerveSubsystemConstants.kLeftRearModuleDriveMotorReversed,
  Constants.SwerveSubsystemConstants.kLeftRearModuleRotationMotorReversed,
  Constants.SwerveSubsystemConstants.kLeftRearModuleDriveEncoderReversed,
  Constants.SwerveSubsystemConstants.kLeftRearModuleRotationEncoderReversed,
  Constants.SwerveSubsystemConstants.kLeftRearModuleCANCoderReversed);
  
  SwerveModule rightFrontModule = new SwerveModule(
  Constants.SwerveSubsystemConstants.kRightFrontModuleDriveMotorID,
  Constants.SwerveSubsystemConstants.kRightFrontModuleRotationMotorID,
  Constants.SwerveSubsystemConstants.kRightFrontModuleCANCoderID,
  Constants.SwerveSubsystemConstants.kRightFrontModuleCANCoderOffsetDegrees,
  Constants.SwerveSubsystemConstants.kRightFrontModuleDriveMotorReversed,
  Constants.SwerveSubsystemConstants.kRightFrontModuleRotationMotorReversed,
  Constants.SwerveSubsystemConstants.kRightFrontModuleDriveEncoderReversed,
  Constants.SwerveSubsystemConstants.kRightFrontModuleRotationEncoderReversed,
  Constants.SwerveSubsystemConstants.kRightFrontModuleCANCoderReversed
  );

  SwerveModule rightRearModule = new SwerveModule(
  Constants.SwerveSubsystemConstants.kRightRearModuleDriveMotorID,
  Constants.SwerveSubsystemConstants.kRightRearModuleRotationMotorID,
  Constants.SwerveSubsystemConstants.kRightRearModuleCANCoderID,
  Constants.SwerveSubsystemConstants.kRightRearModuleCANCoderOffsetDegrees,
  Constants.SwerveSubsystemConstants.kRightRearModuleDriveMotorReversed,
  Constants.SwerveSubsystemConstants.kRightRearModuleRotationMotorReversed,
  Constants.SwerveSubsystemConstants.kRightRearModuleDriveEncoderReversed,
  Constants.SwerveSubsystemConstants.kRightRearModuleRotationEncoderReversed,
  Constants.SwerveSubsystemConstants.kRightRearModuleCANCoderReversed
  );

  public Pigeon2 pigeon = new Pigeon2(Constants.SwerveSubsystemConstants.pigeonID);

  Pigeon2SimState pigeon2SimState = new Pigeon2SimState(pigeon);


  

  public void setStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.kPhysicalMaxSpeed);
  
    leftFrontModule.setDesiredState(states[0]);
    leftRearModule.setDesiredState(states[1]);
    rightFrontModule.setDesiredState(states[2]);
    rightRearModule.setDesiredState(states[3]);

  }

  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        Constants.SwerveSubsystemConstants.kinematics,
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          leftFrontModule.getModulePosition(),
          leftRearModule.getModulePosition(),
          rightFrontModule.getModulePosition(),                                                                 
          rightRearModule.getModulePosition()
         },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  
  public ChassisSpeeds driveRelativeToRobot(){
    return Constants.SwerveSubsystemConstants.kinematics.toChassisSpeeds(getStates());
    
    //speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2D());
    /*SwerveModuleState[] states = Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);
    setStates(states);*/
  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    var states = Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.kPhysicalMaxSpeed);
    setStates(states);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, Constants.ModuleConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.ModuleConstants.kPhysicalMaxSpeed);
    
    setStates(swerveModuleStates);
    
  }


  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFrontModule.getState();
    states[1] = leftRearModule.getState();
    states[2] = rightFrontModule.getState();
    states[3] = rightRearModule.getState();
    return states;
  }

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveSubsystemConstants.kinematics, getRotation2D(), getPositions());

  public ChassisSpeeds getSpeed(){
    return Constants.SwerveSubsystemConstants.kinematics.toChassisSpeeds(getStates());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFrontModule.getModulePosition();
    positions[1] = leftRearModule.getModulePosition();
    positions[2] = rightFrontModule.getModulePosition();
    positions[3] = rightRearModule.getModulePosition();
    return positions;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2D(), getPositions(), pose);
  }

  public void stopModules(){
    leftFrontModule.stop();
    leftRearModule.stop();
    rightFrontModule.stop();
    rightRearModule.stop();
  }
  
  public double getHeading() {
    return Math.IEEEremainder(pigeon.getYaw().getValueAsDouble(), 360);
}

  
  public Rotation2d getRotation2D(){
    return Rotation2d.fromDegrees(-pigeon.getYaw().getValueAsDouble());
    //return Rotation2d.fromDegrees(getHeading());
  }
  
  public void resetHeading(){
    pigeon.reset();
  }


   public SwerveSubsystem() {
    /*new Thread(() -> {
      try{Thread.sleep(1000);
        resetHeading();
      }
      catch(Exception e){

      }
    }).start();*/
    
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
   AutoBuilder.configure(
    this::getPoseEstimatorPose, 
    this::resetPoseEstimatorPose, 
    this::getSpeed, 
    this::drive, 
    new PPHolonomicDriveController(
        Constants.SwerveSubsystemConstants.translationConstants,
        Constants.SwerveSubsystemConstants.rotationConstants
    ),
    config, // 👈 şimdi AUTO SPEED buradan geliyor
    () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    },
    this
);
  }

  public Pose2d getPoseEstimatorPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPoseEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2D(), getPositions(), pose);
  }

  Field2d saha = new Field2d();
  @Override
  public void periodic() {
    //System.out.println("odometrypose" + odometry.getPoseMeters());
      double voltage = RobotController.getBatteryVoltage();

        SmartDashboard.putNumber("Battery Voltage", voltage);

        if (voltage > 12.0) {
            SmartDashboard.putString("Battery Status", "FULL");
        } else if (voltage > 11.0) {
            SmartDashboard.putString("Battery Status", "OK");
        } else {
            SmartDashboard.putString("Battery Status", "LOW ");
        }
    
    odometry.update(getRotation2D(), getPositions());
    saha.setRobotPose(getPose());
    SmartDashboard.putData("field", saha );
    SmartDashboard.putNumber("Left Front Drive Encoder", leftFrontModule.getDrivePosition());
    SmartDashboard.putNumber("Left Rear Drive Encoder", leftRearModule.getDrivePosition());
    SmartDashboard.putNumber("Right Front Drive Encoder", rightFrontModule.getDrivePosition());
    SmartDashboard.putNumber("Right Rear Drive Encoder", rightRearModule.getDrivePosition());
    SmartDashboard.putNumber("Left Rear Angle Encoder", leftRearModule.getRotationPosition());
    SmartDashboard.putNumber("Right Front Angle Encoder", rightFrontModule.getRotationPosition());
    SmartDashboard.putNumber("Right Rear Angle Encoder", rightRearModule.getRotationPosition());
    SmartDashboard.putNumber("Left Front Angle Encoder", leftFrontModule.getRotationPosition() );
    SmartDashboard.putNumber("Left front absolute", leftFrontModule.getAbsoluteAngle());
    SmartDashboard.putNumber("Left rear abs", leftRearModule.getAbsoluteAngle());
    SmartDashboard.putNumber("Right front abs", rightFrontModule.getAbsoluteAngle());
    SmartDashboard.putNumber("Right rear absOLUT", rightRearModule.getAbsoluteAngle());
    SmartDashboard.putNumber("Odometry x ", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Angle", getHeading());
    //SmartDashboard.putNumber("distance horizontal", ll.calculateHorizontalDisntace(getHeading()));
    // This method will be called once per scheduler run
  }
}
