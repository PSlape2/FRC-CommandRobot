package frc.robot.subsystems;

public class InputSystem {

    /*
     * Calculates the input with the curve. Uses the formula for logistic (inhibited) growth.
     * 
     * @param curve the amount to curve the input values
     */
    public static double calculateInputWithCurve(double input, double curveConstant) {
        return 1 / (1 + (curveConstant * Math.pow(Math.E, curveConstant * input)));
    }
}
