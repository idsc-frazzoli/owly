// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.tree;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public interface Distancer {
  /** vector 2-norm */
  static final Distancer EUCLIDEAN = (d1, d2) -> Norm._2.of(d1.subtract(d2));
  /** vector 2-norm squared */
  static final Distancer EUCLIDEAN_SQUARED = (d1, d2) -> Norm._2Squared.of(d1.subtract(d2));

  Scalar getDistance(Tensor d1, Tensor d2);
}