package frc.robot.subsystems;

import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private static final SwerveModulePosition[][] SwerveModulePosition = null;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public PositionVoltage anglePosition;
    public PositionVoltage drivePosition;
    public TalonFXConfiguration swerveAngleFXConfig;
    public Translation2d trans;
    public double rot;
    public boolean fieldRelative;

    public Swerve() {
        gyro = new AHRS(I2C.Port.kMXP);
        gyro.zeroYaw();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        /*AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0,0.0,0.0),
                new PIDConstants(5.0,0.0,0.0),
                4.5,
                0.4,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                } 
                else{
                    return false;
                }
            },
             this
        
        );*/
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //fieldRelative = false;
        trans = translation;
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation/2, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation/2)
                                );
        SmartDashboard.putNumber("X: ", translation.getX());
        SmartDashboard.putNumber("Y: ", translation.getY());
        SmartDashboard.putNumber("rotation", rotation);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(this.getRotation2d(), SwerveModulePosition[0] , pose);
        swerveOdometry.resetPosition(this.getRotation2d(), SwerveModulePosition[1] , pose);
        swerveOdometry.resetPosition(this.getRotation2d(), SwerveModulePosition[2] , pose);
        swerveOdometry.resetPosition(this.getRotation2d(), SwerveModulePosition[3] , pose);
    }
    

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
   
    public void resetPose(Pose2d pose){
        swerveOdometry.resetPosition(resetGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public Rotation2d getRotation2d() {
        return getHeading();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }
    
    public Rotation2d resetGyroYaw(){
        return Rotation2d.fromDegrees(0);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return new ChassisSpeeds(trans.getX(),
        trans.getY(),
        rot/2);
    }

    
    public void auto(){
        anglePosition = new PositionVoltage(0); 
        drivePosition = new PositionVoltage(0);
        RobotContainer.s_Swerve.mSwerveMods[0].mAngleMotor.setControl(anglePosition.withPosition(0));
        RobotContainer.s_Swerve.mSwerveMods[1].mAngleMotor.setControl(anglePosition.withPosition(0));
        RobotContainer.s_Swerve.mSwerveMods[2].mAngleMotor.setControl(anglePosition.withPosition(0));
        RobotContainer.s_Swerve.mSwerveMods[3].mAngleMotor.setControl(anglePosition.withPosition(0));
        RobotContainer.s_Swerve.mSwerveMods[0].mDriveMotor.setControl(drivePosition.withPosition(9));
        RobotContainer.s_Swerve.mSwerveMods[1].mDriveMotor.setControl(drivePosition.withPosition(9));
        RobotContainer.s_Swerve.mSwerveMods[2].mDriveMotor.setControl(drivePosition.withPosition(9));
        RobotContainer.s_Swerve.mSwerveMods[3].mDriveMotor.setControl(drivePosition.withPosition(9));

    }   


    @Override
    public void periodic(){
        
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
        }
    }
    public void stopModules() {
        this.mSwerveMods[0].mAngleMotor.stopMotor();
        this.mSwerveMods[0].mDriveMotor.stopMotor();
        this.mSwerveMods[1].mAngleMotor.stopMotor();
        this.mSwerveMods[1].mDriveMotor.stopMotor();
        this.mSwerveMods[2].mAngleMotor.stopMotor();
        this.mSwerveMods[2].mDriveMotor.stopMotor();
        this.mSwerveMods[3].mAngleMotor.stopMotor();
        this.mSwerveMods[3].mDriveMotor.stopMotor();
        
    }   
}
