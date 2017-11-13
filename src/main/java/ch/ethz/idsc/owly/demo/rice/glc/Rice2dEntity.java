// code by jph
package ch.ethz.idsc.owly.demo.rice.glc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.rice.Rice2GoalManager;
import ch.ethz.idsc.owly.demo.rice.Rice2StateSpaceModel;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractCircularEntity;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.red.Norm2Squared;

/* package */ class Rice2dEntity extends AbstractCircularEntity {
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { { .3, 0, 1 }, { -.1, -.1, 1 }, { -.1, +.1, 1 } }).unmodifiable();
  private static final Integrator INTEGRATOR = MidpointIntegrator.INSTANCE;
  // ---
  private final Collection<Flow> controls;
  public Scalar delayHint = RealScalar.ONE;

  /** @param state initial position of entity */
  public Rice2dEntity(Scalar mu, Tensor state, Collection<Flow> controls) {
    super(new SimpleEpisodeIntegrator(Rice2StateSpaceModel.of(mu), INTEGRATOR, //
        new StateTime(state, RealScalar.ZERO)));
    this.controls = controls;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return Norm2Squared.between(x, y);
  }

  @Override
  protected Tensor fallbackControl() {
    return Tensors.vector(0, 0).unmodifiable();
  }

  @Override
  public final Scalar delayHint() {
    return delayHint;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    Tensor partitionScale = Tensors.vector(3, 3, 6, 6);
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(INTEGRATOR, RationalScalar.of(1, 12), 4);
    Tensor center = Join.of(goal.extract(0, 2), AngleVector.of(goal.Get(2)).multiply(RealScalar.of(0.8)));
    GoalInterface goalInterface = Rice2GoalManager.create(center, Tensors.vector(0.5, 0.5, 0.4, 0.4));
    return new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, goalInterface);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    super.render(geometricLayer, graphics);
    {
      geometricLayer.pushMatrix(geometricLayer.getMouseSe2Matrix());
      graphics.setColor(new Color(0, 128, 255, 192));
      graphics.fill(geometricLayer.toPath2D(SHAPE));
      geometricLayer.popMatrix();
    }
  }
}
