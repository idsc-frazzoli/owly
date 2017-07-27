// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.ArgMin;
import ch.ethz.idsc.tensor.red.Norm;

public class R2Entity implements AnimationInterface, RenderInterface {
  StateSpaceModel ssm = SingleIntegratorStateSpaceModel.INSTANCE;
  EpisodeIntegrator episodeIntegrator = new SimpleEpisodeIntegrator( //
      ssm, //
      EulerIntegrator.INSTANCE, //
      new StateTime(Tensors.vector(0, 0), RealScalar.ZERO));
  // ---
  List<TrajectorySample> trajectory = null;

  void setTrajectory(List<TrajectorySample> trajectory) {
    this.trajectory = trajectory;
  }

  @Override
  public void integrate(Scalar now) {
    Tensor u = Tensors.vector(0, 0);
    if (trajectory != null) {
      final Tensor x = episodeIntegrator.tail().x();
      Tensor dif = Tensor.of( //
          trajectory.stream().map(ts -> Norm._2.of(ts.stateTime().x().subtract(x))));
      int index = ArgMin.of(dif);
      // TODO negate due to reverse trajectory ... not final solution ...
      if (0 <= index && trajectory.get(index).getFlow().isPresent())
        u = trajectory.get(index).getFlow().get().getU().negate();
    }
    episodeIntegrator.move(u, now);
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    StateTime stateTime = episodeIntegrator.tail();
    Point2D p = owlyLayer.toPoint2D(stateTime.x());
    graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
    graphics.fill(new Rectangle2D.Double(p.getX() - 2, p.getY() - 2, 5, 5));
  }
}
