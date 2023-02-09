package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveChassisSubsystem extends SubsystemBase {
    
   /*private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftEncChanA,
            DriveConstants.kFrontLeftEncChanB
            
    );*/

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightEncChanA,
            DriveConstants.kFrontRightEncChanB
            
            );

    /*private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftEncChanA,
            DriveConstants.kBackLeftEncChanB
            
            );

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightEncChanA,
            DriveConstants.kBackLeftEncChanB
            
            );
    */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            getRotation2d(), getSwerveModulePositions());

    public SwerveChassisSubsystem() {
    
        new Thread(() -> {
            try {
                //Delays 1 second before resetting heading
                //Allows gyro to recalibrate first on boot
                //Is on a seperate thread so the rest of the code is unaffected by sleep(1000)
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    //Resets gyro's heading on boot to recalibrate what it considers forwards
    public void zeroHeading() {
        gyro.reset();
    }
    
    //Gyro's value is continuous, it can go past 360
    //This function clamps it between 180 and -180 degrees to make it easier to work with
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //Converts gyro heading into Rotation2d so the robot can use it
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[]{
            //frontLeft.getSMPosition(),
            frontRight.getSMPosition(),
            //backLeft.getSMPosition(),
            //backRight.getSMPosition()

        };
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    @Override
    public void periodic() {
        //odometer.update(getRotation2d(), getSwerveModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("drive enc", frontRight.getDrivePosition());
        SmartDashboard.putNumber("turn enc", frontRight.getTurningPosition());
        SmartDashboard.putNumber("abs enc", frontRight.getAbsoluteEncoderRad());
    }

    public void stopModules() {
        //frontLeft.stop();
        frontRight.stop();
        //backLeft.stop();
        //backRight.stop();
    }

    //Takes in an array of the desired states for each swerve module
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        //backLeft.setDesiredState(desiredStates[2]);
        //backRight.setDesiredState(desiredStates[3]);
    }
}
