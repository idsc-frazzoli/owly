// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EmptyRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
public class Se2Demo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    Scalar timeStep = RationalScalar.of(1, 6);
    Tensor partitionScale = Tensors.vector(3, 3, 15); // .multiply(resolutionFactor); //
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), 6);
    int trajectorySize = 5;
    CostFunction costFunction = new MinTimeCost();
    Se2Goal rnGoal = new Se2Goal( //
        Tensors.vector(2, 1), RealScalar.of(Math.PI), //
        DoubleScalar.of(.75), Se2Utils.DEGREE(10));
    Heuristic heuristic = rnGoal; // new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            rnGoal));
    // new EllipsoidRegion(Tensors.vector(5, -1, Math.PI/2), Tensors.vector(.5, .5, .2)) //
    // ));
    TrajectoryRegionQuery obstacleQuery = //
        new EmptyRegionQuery();
    // new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
    // RegionUnion.of( //
    // new EllipsoidRegion(Tensors.vector(+3, +1), Tensors.vector(1.75, .75)), // speed limit along the way
    // new EllipsoidRegion(Tensors.vector(-2, +0), Tensors.vector(1, 1)) // block to the left
    // )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = trajectoryPlanner.plan(2000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectory.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
