// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EmptyRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.IntegrationConfig;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Pendulum Swing-up
 * 
 * implementation inspired by
 * "A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning" [Paden/Frazzoli] */
public class PsuDemo {
  public static void main(String[] args) {
    Scalar timeStep = RationalScalar.of(1, 4);
    Tensor partitionScale = Tensors.vector(5, 7);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuGoalManager psuGoalManager = new PsuGoalManager(Tensors.vector(.1, .1));
    TrajectoryRegionQuery obstacleQuery = new EmptyRegionQuery();
    IntegrationConfig integrationConfig = IntegrationConfig.createDefault(timeStep, 5);
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrationConfig, partitionScale, controls, //
        psuGoalManager, psuGoalManager, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
