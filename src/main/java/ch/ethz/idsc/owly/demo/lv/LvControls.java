// code by jph
package ch.ethz.idsc.owly.demo.lv;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensors;

public class LvControls {
  public static Collection<Flow> set() {
    StateSpaceModel stateSpaceModel = new LvStateSpaceModel();
    return Arrays.asList( //
        StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(0)));
  }
}
