// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.DeleteDuplicates;
import ch.ethz.idsc.tensor.red.Norm;
import junit.framework.TestCase;

public class TwdControlsTest extends TestCase {
  public void testSimple() {
    Scalar wheelRadius = RationalScalar.of(5, 100); // 5[cm]
    Scalar wheelDistance = RationalScalar.of(40, 10); // 40[cm]
    StateSpaceModel ssm = new TwdStateSpaceModel(wheelRadius, wheelDistance);
    List<Flow> list = (List<Flow>) TwdControls.createControls(ssm, 4);
    Tensor us = Tensors.empty();
    for (Flow flow : list) {
      Tensor u = flow.getU();
      Scalar norm = Norm.INFINITY.of(u);
      assertEquals(norm, RealScalar.ONE);
      us.append(u);
    }
    Tensor unique = DeleteDuplicates.of(us);
    assertEquals(unique.length(), list.size());
  }
}
