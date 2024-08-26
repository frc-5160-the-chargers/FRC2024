package frc4481;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Credits: 4481 repository
 */
@SuppressWarnings("unused")
public class HeadingCorrector {
    double previousT;
    double offT;
    Timer timer = new Timer();

    private Rotation2d targetHeading;
    private Rotation2d getTargetHeading(){
        return targetHeading;
    }
    private void setTargetHeading(Rotation2d theHeading) {
        targetHeading = theHeading;
    }

    public ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed, Rotation2d inputYaw){
        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;
        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);
        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            setTargetHeading(inputYaw);
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            setTargetHeading(inputYaw);
            return desiredSpeed;
        }
        if (v < 0.1){
            setTargetHeading(inputYaw);
            return desiredSpeed;
        }
        //Determine target and current heading
        setTargetHeading( getTargetHeading().plus(new Rotation2d(vr * dt)) );
        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = getTargetHeading().minus(inputYaw);
        if (Math.abs(deltaHeading.getDegrees()) < 0.05){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * 0.05;
        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }
}
