package frc.team1699.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.SwerveConstants;
import frc.team1699.Constants.SwerveModuleConstants.BL;
import frc.team1699.Constants.SwerveModuleConstants.BR;
import frc.team1699.Constants.SwerveModuleConstants.FL;
import frc.team1699.Constants.SwerveModuleConstants.FR;
import frc.team1699.Constants.SwerveModuleConstants.SwerveGyroConstants;
import frc.team1699.lib.SwerveGyro;
import frc.team1699.lib.SwerveModule;

public class Swerve {
    private SwerveModule[] modules;
    private SwerveGyro gyro;

    public Swerve() {
        this.modules = new SwerveModule[] {
            new SwerveModule(FL.kDriveID, FL.kDriveInverted, FL.kAngleID, FL.kAngleInverted, FL.kEncoderID, FL.kEncoderInverted, FL.kEncoderOffset),
            new SwerveModule(FR.kDriveID, FR.kDriveInverted, FR.kAngleID, FR.kAngleInverted, FR.kEncoderID, FR.kEncoderInverted, FR.kEncoderOffset),
            new SwerveModule(BL.kDriveID, BL.kDriveInverted, BL.kAngleID, BL.kAngleInverted, BL.kEncoderID, BL.kEncoderInverted, BL.kEncoderOffset),
            new SwerveModule(BR.kDriveID, BR.kDriveInverted, BR.kAngleID, BR.kAngleInverted, BR.kEncoderID, BR.kEncoderInverted, BR.kEncoderOffset)
        };
        this.gyro = new SwerveGyro(SwerveGyroConstants.kGyroOffset, SwerveGyroConstants.kGyroInverted);
    }

    public void driveXbox(XboxController controller) {
        double x = controller.getLeftX();
        x = MathUtil.applyDeadband(x, SwerveConstants.kDeadband);
        double y = controller.getLeftY();
        y = MathUtil.applyDeadband(y, SwerveConstants.kDeadband);
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = new Rotation2d(x, y);
        linearMagnitude *= SwerveConstants.kMaxVelocity;
        Translation2d linearVelocity = 
            new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d()))
            .getTranslation();

        double rotation = controller.getRightX();
        rotation = MathUtil.applyDeadband(rotation, SwerveConstants.kDeadband);
        rotation *= SwerveConstants.kMaxAngularVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), rotation);
        if(SwerveConstants.fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

        for(int i = 0; i < 4; i++) {
            modules[i].setModuleState(states[i]);
        }
    }

    public void stop() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
        for(int i = 0; i < 4; i++) {
            modules[i].setModuleState(states[i]);
        }
    }
}
