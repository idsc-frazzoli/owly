// code by jph
package ch.ethz.idsc.owly.gui.ani;

import ch.ethz.idsc.owly.demo.rice.Rice2StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

public class Rice2Animated implements AnimationInterface {
  StateSpaceModel ssm = new Rice2StateSpaceModel(RealScalar.of(1.5));

  EpisodeIntegrator episodeIntegrator = new SimpleEpisodeIntegrator( //
      ssm, //
      EulerIntegrator.INSTANCE, //
      new StateTime(Tensors.vector(0, 0), RealScalar.ZERO));

  @Override
  public void integrate(Scalar now) {
    episodeIntegrator.move(Tensors.vector(.3), now);
    StateTime st = episodeIntegrator.tail();
    System.out.println(st.x());
  }
}
