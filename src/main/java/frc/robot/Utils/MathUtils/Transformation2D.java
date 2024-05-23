package frc.robot.Utils.MathUtils;

public class Transformation2D {
    /** the original unstressed space */
    public static Transformation2D getOriginalSpace() {
        /*
        * don't do static final Transformation2D = ...
        * the pointer will be the same
        */
        return new Transformation2D(new double[] { 1, 0 },
                new double[] { 0, 1 });
    }
    /** the value of this transformation */
    private Vector2D[] value;

    public Transformation2D() {
        value = getOriginalSpace().getValue();
    }

    public Transformation2D(double[][] initialValue) {
        value = new Vector2D[] { new Vector2D(initialValue[0]), new Vector2D(initialValue[1]) };
    }

    public Transformation2D(Vector2D iHat, Vector2D jHat) {
        value = new Vector2D[] { iHat, jHat };
    }

    public Transformation2D(double[] iHat, double[] jHat) {
        value = new Vector2D[] { new Vector2D(iHat), new Vector2D(jHat) };
    }

    public void setIHat(double[] iHat) {
        value[0] = new Vector2D(iHat);
    }

    public void setIHat(Vector2D iHat) {
        value[0] = iHat;
    }

    public void setJHat(double[] jHat) {
        value[1] = new Vector2D(jHat);
    }

    public void setJHat(Vector2D jHat) {
        value[1] = jHat;
    }

    public Vector2D[] getValue() {
        return value;
    }

    public Vector2D getIHat() {
        return value[0];
    }

    public Vector2D getJHat() {
        return value[1];
    }

    /** apply this transformation to a given vector */
    public Vector2D multiply(Vector2D vector) {
        Vector2D transformedVector = new Vector2D();
        double x = vector.getValue()[0];
        double y = vector.getValue()[1];
        Vector2D iHat = value[0];
        Vector2D jHat = value[1];

        transformedVector = transformedVector.addBy(iHat.multiplyBy(x));
        transformedVector = transformedVector.addBy(jHat.multiplyBy(y));

        return transformedVector;
    }

    /** find the determinant of the current transformation */
    public double getDeterminant() {
        double[] iHatValues = value[0].getValue();
        double[] jHatValues = value[1].getValue();
        return iHatValues[0] * jHatValues[1] - iHatValues[1] * jHatValues[0];
    }

    /*
     * in reference to the code of chat gpt
     * if (determinant != 0) {
     * double[][] reversalValues = {
     * { value[1][1] / determinant, -value[0][1] / determinant },
     * { -value[1][0] / determinant, value[0][0] / determinant }
     * };
     *
     * return new Transformation2D(reversalValues);
     * } else {
     * throw new UnsupportedOperationException("no reverse for this matrix");
     * }
     */
    /** find the reversal of the current transformation */
    public Transformation2D getReversal() {
        double determinant = getDeterminant();

        if (determinant == 0)
            throw new UnsupportedOperationException("The given transformation have no reverse");
        double[] reversalIHatValue = { value[1].getValue()[1] / determinant, -value[0].getValue()[1] / determinant };
        double[] reversalJHatValue = { -value[1].getValue()[0] / determinant, value[0].getValue()[0] / determinant };
        Transformation2D reversalTransformation = new Transformation2D(reversalIHatValue, reversalJHatValue);
        return reversalTransformation;
    }

    /** turn the transformation into string */
    @Override
    public String toString() {
        return String.format("transformation with value: \n [ %.2f  %.2f] \n [%.2f  %.2f]",
                value[0].getValue()[0], value[1].getValue()[0], value[0].getValue()[1], value[1].getValue()[1]);
    }

    /** judge if another transformation is equal to this transformation */
    @Override
    public boolean equals(Object comparison) {
        Transformation2D comparisonTransformation = (Transformation2D) comparison;
        return comparisonTransformation.getIHat().equals(this.getIHat())
                && comparisonTransformation.getJHat().equals(this.getJHat());
    }
}
