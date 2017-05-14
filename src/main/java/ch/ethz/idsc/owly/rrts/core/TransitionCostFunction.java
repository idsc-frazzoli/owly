// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Scalar;

public interface TransitionCostFunction {
  Scalar cost(Transition transition);
}
