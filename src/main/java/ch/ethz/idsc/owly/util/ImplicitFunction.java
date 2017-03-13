// code by jph
package ch.ethz.idsc.owly.util;

import java.util.function.Function;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

public interface ImplicitFunction extends Function<Tensor, RealScalar> {
  // ---
}
