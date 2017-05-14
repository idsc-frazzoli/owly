// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import java.util.function.Function;

import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionCostFunction;
import ch.ethz.idsc.tensor.Scalar;

public class LengthCostFunction implements TransitionCostFunction {
  public static final TransitionCostFunction IDENTITY = new LengthCostFunction(Function.identity());

  public static TransitionCostFunction create(Function<Scalar, Scalar> function) {
    return new LengthCostFunction(function);
  }

  private final Function<Scalar, Scalar> function;

  private LengthCostFunction(Function<Scalar, Scalar> function) {
    this.function = function;
  }

  @Override
  public Scalar cost(Transition transition) {
    return function.apply(transition.length());
  }
}
