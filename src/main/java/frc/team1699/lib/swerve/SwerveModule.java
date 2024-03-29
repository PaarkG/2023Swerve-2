package frc.team1699.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1699.Constants.SwerveModuleConstants;

public class SwerveModule {
    private CANSparkMax driveMotor;
    private PIDController driveController;
    private RelativeEncoder driveEncoder;
    private CANSparkMax angleMotor;
    private PIDController angleController;
    private SwerveEncoder angleEncoder;
    private Rotation2d lastAngle;

    public SwerveModule(int driveID, boolean driveInverted, int angleID, boolean angleInverted, int encoderID, boolean encoderInverted, double offset) {
        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setInverted(driveInverted);
        this.driveController = new PIDController(.002, 0, 0);
        this.driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDistancePerMotorRotation);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDistancePerMotorRotation);
        this.angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        angleMotor.setInverted(angleInverted);
        this.angleController = new PIDController(.01, 0, 0);
        this.angleEncoder = new SwerveEncoder(encoderID, offset, encoderInverted);
        this.lastAngle = angleEncoder.getRotation2d();
    }

    public void setModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, lastAngle);
        double driveOutput = driveController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        double angleOutput = angleController.calculate(angleEncoder.getRotation2d().getRadians(), state.angle.getRadians());
        driveMotor.set(driveOutput);
        angleMotor.set(angleOutput);
        lastAngle = state.angle;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), angleEncoder.getRotation2d());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), angleEncoder.getRotation2d());
    }
}