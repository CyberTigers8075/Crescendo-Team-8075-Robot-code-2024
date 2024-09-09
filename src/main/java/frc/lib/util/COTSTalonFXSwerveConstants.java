package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSTalonFXSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;
    public final InvertedValue rightDriveInverted;
    public final InvertedValue leftDriveInverted; 

    public COTSTalonFXSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, InvertedValue angleMotorInvert, SensorDirectionValue cancoderInvert, InvertedValue rightDriveInverted, InvertedValue leftDriveInverted){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
        this.rightDriveInverted = rightDriveInverted;
        this.leftDriveInverted = leftDriveInverted;
    
    }


    /** Swerve Drive Specialities */
    public static final class SDS {
       
    
        /** Swerve Drive Specialties - MK4 Module*/
        public static final class MK4{
            /** Swerve Drive Specialties - MK4 Module (Falcon 500)*/
            public static final COTSTalonFXSwerveConstants Falcon500(double driveGearRatio){
                double wheelDiameter = Units.inchesToMeters(4.0);
        
                /** 12.8 : 1 */
                double angleGearRatio = (12.8 / 1.0);
        
                double angleKP = 24.0;
                double angleKI = 0.0;
                double angleKD = 0.1;
        
    
                    InvertedValue rightDriveMotor = InvertedValue.Clockwise_Positive;
                    InvertedValue leftDriveMotor = InvertedValue.CounterClockwise_Positive;
                    InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
                    SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
                    return new COTSTalonFXSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleMotorInvert, cancoderInvert, rightDriveMotor, leftDriveMotor);
                }

           
            public static final class driveRatios{
                /** SDS MK4 - (6.75 : 1) */
                public static final double L2 = (6.75 / 1.0);
                /** SDS MK4 - (6.12 : 1) */
            }
        }
    
    }
}

  