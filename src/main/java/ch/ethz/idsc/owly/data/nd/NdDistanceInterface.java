// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public interface NdDistanceInterface {
  /** vector 2-norm */
  static final NdDistanceInterface EUCLIDEAN = (d1, d2) -> Norm._2.of(d1.subtract(d2));
  /** vector 2-norm squared */
  static final NdDistanceInterface EUCLIDEAN_SQUARED = (d1, d2) -> Norm._2Squared.of(d1.subtract(d2));

  Scalar apply(Tensor d1, Tensor d2);
}