// code by jph
package ch.ethz.idsc.owly.math;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface CoordinateWrap extends Serializable {
  Tensor represent(Tensor x);

  Scalar distance(Tensor p, Tensor q);
}
