package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.math.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.io.ExtractPrimitives;

/** numerous ~circles of varying size
 * the challenge is to find the path through narrow passages as close as possible to the diagonal
 *
 * suggested values for use are
 * extent = 2
 * root = ( -2.0, -2.0 )
 * goal = ( 2.0, 2.0 ) */
public class R2Bubbles extends ImplicitFunctionRegion {
  @Override
  public Scalar apply(Tensor tensor) {
    final double[] data = ExtractPrimitives.toArrayDouble(tensor);
    double x = data[0];
    double y = data[1];
    double x2 = x * x;
    double y2 = y * y;
    double val = 0;
    val -= Math.cos(3 * Math.PI * x);
    val -= Math.cos(3 * Math.PI * y);
    val += -0.2 * (1 + y2 + x * y + x2);
    return RealScalar.of(-val);
  }
}
