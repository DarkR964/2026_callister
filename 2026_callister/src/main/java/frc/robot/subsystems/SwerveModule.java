package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private final CANcoder cancoder;

    private final PIDController anglePID;

    private final double cancoderOffset;
    private final boolean cancoderReversed;
    private final boolean driveEncoderReversed;
    private final boolean rotationEncoderReversed;

    public SwerveModule(
        int driveMotorID,
        int rotationMotorID,
        int cancoderID,
        double cancoderOffsetDegrees,
        boolean driveMotorReversed,
        boolean rotationMotorReversed,
        boolean driveEncoderReversed,
        boolean rotationEncoderReversed,
        boolean cancoderReversed
    ) {

        this.cancoderOffset = cancoderOffsetDegrees;
        this.cancoderReversed = cancoderReversed;
        this.driveEncoderReversed = driveEncoderReversed;
        this.rotationEncoderReversed = rotationEncoderReversed;

        driveMotor = new TalonFX(driveMotorID);
        rotationMotor = new TalonFX(rotationMotorID);
        cancoder = new CANcoder(cancoderID);

        anglePID = new PIDController(Constants.ModuleConstants.kRotationP, 0, 0);
        anglePID.enableContinuousInput(-180, 180);
        
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted =
            driveMotorReversed
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rotationConfig.MotorOutput.Inverted =
            rotationMotorReversed
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        rotationMotor.getConfigurator().apply(rotationConfig);

        resetEncoders();
    }

    public double getAbsoluteAngle() {
        double angle = cancoder.getAbsolutePosition().getValue().in(Degrees);
        angle -= cancoderOffset;
        angle *= cancoderReversed ? -1.0 : 1.0;
        return angle;
    }

    public double getRotationPosition() {
        return rotationMotor.getPosition().getValueAsDouble()
            * Constants.ModuleConstants.kRotationEncoderPositionConversionFactor
            * (rotationEncoderReversed ? -1.0 : 1.0);
    }

    public double getRotationVelocity() {
        return rotationMotor.getVelocity().getValueAsDouble()
            * Constants.ModuleConstants.kRotationEncoderVelocityConversionFactor
            * (rotationEncoderReversed ? -1.0 : 1.0);
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble()
            * Constants.ModuleConstants.kDriveEncoderPositionConversionFactor
            * (driveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble()
            * Constants.ModuleConstants.kDriveEncoderVelocityConversionFactor
            * (driveEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            Rotation2d.fromDegrees(getRotationPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            Rotation2d.fromDegrees(getRotationPosition())
        );
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        rotationMotor.setPosition(
            getAbsoluteAngle()
            / Constants.ModuleConstants.kRotationEncoderPositionConversionFactor
        );
    }

    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.02) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(
            state,
            Rotation2d.fromDegrees(getRotationPosition())
        );

        driveMotor.setControl(
            new DutyCycleOut(
                state.speedMetersPerSecond
                    / Constants.ModuleConstants.kPhysicalMaxSpeed
            )
        );

        double output = anglePID.calculate(
            getRotationPosition(),
            state.angle.getDegrees()
        );

        rotationMotor.setControl(new DutyCycleOut(output));
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }
}
