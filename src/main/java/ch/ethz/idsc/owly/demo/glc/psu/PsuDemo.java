// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import ch.ethz.idsc.owly.glc.adapter.EmptyRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.EulerIntegrator;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.owly.util.rn.RnSphericalRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Pendulum Swing-up
 * 
 * implementation inspired by
 * "A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning" [Paden/Frazzoli] */
public class PsuDemo {
  public static void main(String[] args) {
    StateSpaceModel stateSpaceModel = new PsuIntegrator();
    DynamicalSystem dynamicalSystem = new EulerIntegrator() {
      @Override
      public Scalar getMaxTimeStep() {
        return RealScalar.of(.25);
      }
    };
    Tensor controls = PsuControls.createControls(6);
    // System.out.println(Pretty.of(controls));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    // TODO join +PI and -PI
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new RnSphericalRegion(Tensors.vector(Math.PI, 0), RealScalar.of(1))));
    TrajectoryRegionQuery obstacleQuery = new EmptyRegionQuery();
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        stateSpaceModel, //
        dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.initialize(Tensors.vector(10, 10));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan();
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
