package frc.lib.math;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class Conversions {
    
    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @author Braden Harris Lynn
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        if(Robot.getRedAlliance() && DriverStation.isTeleopEnabled()){
            return -wheelMeters;
        }
        else{
            return wheelMeters;
        }
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
}