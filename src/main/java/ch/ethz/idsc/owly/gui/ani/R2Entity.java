// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

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
import ch.ethz.idsc.tensor.Scalars;
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

  /** @return index of sample of trajectory that is closest to current position */
  private int indexOfClosestTrajectorySample() {
    final Tensor x = episodeIntegrator.tail().state();
    return ArgMin.of(Tensor.of(trajectory.stream() //
        .map(TrajectorySample::stateTime) //
        .map(StateTime::state) //
        .map(state -> Norm._2SQUARED.of(state.subtract(x)))));
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
    // System.out.println(getEstimatedLocationAt(RealScalar.ONE));
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = episodeIntegrator.tail().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(128 - 64, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    { // indicate position 1 sec into the future
      Tensor state = getEstimatedLocationAt(RealScalar.of(1.0));
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }

  /** @param delay
   * @return estimated location of agent after given delay */
  Tensor getEstimatedLocationAt(Scalar delay) {
    if (Objects.isNull(trajectory))
      return episodeIntegrator.tail().state();
    // TODO JAN this code is almost generic => extract to util class
    int index = indexOfClosestTrajectorySample();
    TrajectorySample current = trajectory.get(index);
    Scalar threshold = current.stateTime().time().add(delay);
    Optional<TrajectorySample> optional = trajectory.stream() //
        .filter(ts -> Scalars.lessEquals(threshold, ts.stateTime().time())) //
        .findFirst();
    return optional.orElse(trajectory.get(trajectory.size() - 1)) //
        .stateTime().state();
  }
}
