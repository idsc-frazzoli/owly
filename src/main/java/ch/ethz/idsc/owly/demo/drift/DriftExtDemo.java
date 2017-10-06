// code by edo
package ch.ethz.idsc.owly.demo.drift;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
import ch.ethz.idsc.owly.math.region.NegativeHalfspaceRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DriftExtDemo {
  ;
  public static void main(String[] args) throws IOException {
    // the resolution refers to the last 3 of the state coordinates (x,y,theta,beta,r,Ux)
    Tensor eta = Tensors.vector(30, 30, 5); // magic const
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.createDefault(RationalScalar.of(1, 10), 7);
    System.out.println("scale=" + eta);
    Collection<Flow> controls = DriftControls.createExtended(10); // magic const
    GoalInterface goalInterface = DriftGoalManager.createStandard(//
        Tensors.vector(0, 0, 0, -0.3055, 0.5032, 8), //
        Tensors.vector( //
            Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, //
            0.05, 0.05, 0.25));
    // ---
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new NegativeHalfspaceRegion(4) // ensure that r is non-negative
                , // impose that x < Threshold
                new HyperplaneRegion(Tensors.vector(-1, 0, 0, 0, 0, 0), RealScalar.of(12)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.represent = x -> x.extract(3, 6); // consider only (beta,r,Ux)
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0, 0, 0, 1));
    int iters = Expand.maxSteps(trajectoryPlanner, 10000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<GlcNode> trajectory = Nodes.listFromRoot(optional.get());
      GlcNodeExport glcNodeExport = new GlcNodeExport("t, x, y, theta, beta, r, Ux, delta, Fx");
      for (GlcNode node : trajectory) {
        if (!node.isRoot())
          System.out.println(node.flow().getU());
        System.out.println(node.stateTime().toCompactString());
        glcNodeExport.append(node);
      }
      // StateTimeTrajectories.print(trajectory);
      new File("export").mkdir();
      glcNodeExport.writeToFile(new File("export/drift.csv"));
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
