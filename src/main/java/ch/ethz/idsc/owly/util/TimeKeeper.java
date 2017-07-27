// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public class TimeKeeper {
  private final long tic = System.nanoTime();

  public Scalar now() {
    long toc = System.nanoTime();
    double delta = (toc - tic) * 1e-9;
    return RealScalar.of(delta);
  }
}
