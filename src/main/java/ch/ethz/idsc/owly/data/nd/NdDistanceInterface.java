// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.red.Norm2Squared;

public interface NdDistanceInterface extends Serializable {
  /** vector 2-norm */
  static final NdDistanceInterface EUCLIDEAN = (d1, d2) -> Norm._2.ofVector(d1.subtract(d2));
  /** vector 2-norm squared */
  static final NdDistanceInterface EUCLIDEAN_SQUARED = (d1, d2) -> Norm2Squared.ofVector(d1.subtract(d2));

  Scalar apply(Tensor d1, Tensor d2);
}