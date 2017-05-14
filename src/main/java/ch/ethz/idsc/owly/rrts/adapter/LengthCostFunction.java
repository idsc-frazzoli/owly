// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionCostFunction;
import ch.ethz.idsc.tensor.Scalar;

public class LengthCostFunction implements TransitionCostFunction {
  @Override
  public Scalar cost(Transition transition) {
    return transition.length();
  }
}
