// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Tensor;

public interface Flow {
  Tensor at(Tensor x);
}
