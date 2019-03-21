package frc.team2225.robot;

public final class ScaleInputs {
    private ScaleInputs() {}

    /**
     * A simple exponent which preserves the sign of the input
     * @param base The input value
     * @param exponent The exponent
     * @return base^exponent, with the sign of base
     */
    public static double signedExponent(double base, double exponent) {
        return Math.copySign(Math.pow(Math.abs(base), exponent), base);
    }

    /**
     * Scales an input to make it easier to control a robot with a gamepad
     * @param valIn The input value
     * @param deadzoneRadius The minimum value required to recognize the input as valid
     * @param minValue The minimum value returned if input is valid
     * @param exponent The exponent applied to curve the results
     * @return The scaled output
     */
    // old constants -> double[] param = {0.2, 0.3, 2};
    public static double scaleInputs(double valIn, double deadzoneRadius, double minValue, double exponent) {
        double val = valIn;
        val = deadzone(deadzoneRadius, val);
        val = signedExponent(val, exponent);

        if(Math.abs(val) > 0)
            val = padMinValue(minValue, val, true);
        return val;
    }

    /**
     * Scales an input to make it easier to control a robot with a gamepad
     * This method uses default values
     * @param valIn The input value
     * @return The scaled output
     */
    public static double scaleInputs(double valIn) {
        return scaleInputs(valIn, 0.2, 0.2, 2);
    }

    /**
     * Adds a pad to the value
     * This scales the values from [0, 1] to [0, 1 - pad]
     * If the pad is included, then the output is [pad, 1]
     * @param pad The size of the pad
     * @param value The input value
     * @param includePad Whether to add the pad to the result
     * @return The scaled value
     */
    public static double padMinValue(double pad, double value, boolean includePad) {
        return Math.copySign((1 - pad) * Math.abs(value) + (includePad ? pad : 0), value);
    }

    /**
     * Adds a deadzone to the value
     * If the value is smaller than the deadzone, returns zero
     * If it is bigger, it scales it from [deadzone, 1] -> [0, 1]
     * @param deadzone The deadzone radius (must be positive)
     * @param value The input value [-1, 1]
     * @return The scaled value
     */
    public static double deadzone(double deadzone, double value) {
        if(Math.abs(value) < deadzone)
            return 0;
        return Math.copySign((Math.abs(value) - deadzone)/(1-deadzone), value);
    }


}
