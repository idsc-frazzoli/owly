// code by jl and jph
package ch.ethz.idsc.owly.demo.se2.any;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinDistGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** omni-directional movement with constant speed */
public class SE2AnyEntity extends AbstractAnyEntity {
  // private static final JLabel JLABEL = new JLabel();
  private static final Tensor FALLBACK_CONTROL = Array.zeros(2).unmodifiable(); // {angle=0, vel=0}
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07, 1 }, //
          { .2, -.07, 1 }, //
          { -.1, -.07, 1 }, //
          { -.1, +.07, 1 } //
      }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  // ---
  protected final Collection<Flow> controls = Se2Controls.createControlsForwardAndReverse(RealScalar.ONE, parameters.getResolutionInt());
  private final Tensor goalRadius;

  /** @param state initial position of entity */
  public SE2AnyEntity(Tensor state, int resolution) {
    super(state, //
        // --
        new Se2Parameters( //
            (RationalScalar) RealScalar.of(resolution), // resolution
            RealScalar.of(2), // TimeScale
            RealScalar.of(100), // DepthScale
            Tensors.vector(5, 5, 50 / Math.PI), // PartitionScale 50/pi == 15.9155
            RationalScalar.of(1, 6), // dtMax
            2000, // maxIter
            Se2StateSpaceModel.INSTANCE.getLipschitz()), // Lipschitz
        // --
        Se2Controls.createControlsForwardAndReverse(RotationUtils.DEGREE(60), resolution), //
        // --
        new SimpleEpisodeIntegrator( //
            Se2StateSpaceModel.INSTANCE, //
            RungeKutta4Integrator.INSTANCE, //
            new StateTime(state, RealScalar.ZERO)),
        //
        RealScalar.of(1.5), RealScalar.of(1)); //
    final Scalar goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(0));
    final Scalar goalRadius_theta = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y);
  }

  @Override
  public Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    {// indicate current position
      StateTime stateTime = getStateTimeNow();
      Color color = new Color(64, 64, 64, 128);
      graphics.setColor(color);
      Tensor matrix = Se2Utils.toSE2Matrix(stateTime.state());
      Path2D path2d = owlyLayer.toPath2D(Tensor.of(SHAPE.flatten(0).map(matrix::dot)));
      graphics.fill(path2d);
    }
    { // indicate position delay[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 64, 192));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    {
      graphics.setColor(new Color(0, 128, 255, 192));
      Tensor matrix = owlyLayer.getMouseSe2Matrix();
      Path2D path2d = owlyLayer.toPath2D(Tensor.of(SHAPE.flatten(0).map(matrix::dot)));
      graphics.fill(path2d);
    }
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return new Se2MinDistGoalManager(goal, goalRadius).getGoalInterface();
  }

  @Override
  protected StateIntegrator createIntegrator() {
    return FixedStateIntegrator.create(RungeKutta4Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
  }
}
