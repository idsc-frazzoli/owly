// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface StateSpaceModel {
  Tensor flow(Tensor x, Tensor u);

  Scalar getLipschitz();
}
