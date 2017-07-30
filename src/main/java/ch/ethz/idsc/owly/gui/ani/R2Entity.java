// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owly.data.GlobalAssert;
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
  private List<TrajectorySample> trajectory = null;

  void setTrajectory(List<TrajectorySample> trajectory) {
    this.trajectory = trajectory;
  }

  private int indexOfClosestTrajectorySample() {
    final Tensor x = episodeIntegrator.tail().state();
    Tensor dif = Tensor.of( //
        trajectory.stream().map(ts -> Norm._2.of(ts.stateTime().state().subtract(x))));
    return ArgMin.of(dif);
  }

  @Override
  public void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    Tensor u = Tensors.vector(0, 0); // default control
    if (Objects.nonNull(trajectory)) {
      int index = indexOfClosestTrajectorySample();
      GlobalAssert.that(index != ArgMin.NOINDEX);
      ++index; // next node has flow control
      if (index < trajectory.size()) {
        GlobalAssert.that(trajectory.get(index).getFlow().isPresent());
        u = trajectory.get(index).getFlow().get().getU();
      } else {
        System.out.println("trajectory finished");
        trajectory = null;
      }
    }
    episodeIntegrator.move(u, now);
    // ---
  }

  /** @param future
   * @return estimated location of agent at given future */
  Tensor getEstimatedLocationAt(Scalar future) {
    if (Objects.isNull(trajectory))
      return episodeIntegrator.tail().state();
    return null;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    StateTime stateTime = episodeIntegrator.tail();
    Point2D p = owlyLayer.toPoint2D(stateTime.state());
    graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
    graphics.fill(new Rectangle2D.Double(p.getX() - 2, p.getY() - 2, 5, 5));
  }
}
