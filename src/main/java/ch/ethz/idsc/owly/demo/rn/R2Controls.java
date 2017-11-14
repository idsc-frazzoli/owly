// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collection;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;

// create radial controls
// class is intentionally public 
public enum R2Controls {
  ;
  public static Scalar maxSpeed(Collection<Flow> controls) {
    int length = controls.iterator().next().getU().length();
    GlobalAssert.that(length == 2);
    return controls.stream().map(Flow::getU).map(Norm._2::ofVector).reduce(Max::of).get();
  }
}
