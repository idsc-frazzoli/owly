// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Join;

enum StaticHelper {
  ;
  static StateTimeTensorFunction JOINED = //
      stateTime -> Join.of(stateTime.state(), Tensors.of(stateTime.time()));
}
