// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Arrays;
import java.util.List;

import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owl.gui.ani.AbstractEntity;
import ch.ethz.idsc.owl.gui.ani.AbstractRrtsEntity;
import ch.ethz.idsc.owl.gui.ani.AnimationInterface;
import ch.ethz.idsc.owl.gui.ani.MotionPlanWorker;
import ch.ethz.idsc.owl.gui.ani.OwlySimulation;
import ch.ethz.idsc.owl.img.ImageRegions;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegionWrap;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Se2GlcAnyTrackDemo implements DemoInterface {
  Se2AnyEntity se2AnyEntity;

  @Override
  public void start() {
    OwlySimulation OwlySimulation = new OwlySimulation();
    StateTime root = new StateTime(Tensors.vector(7, 6, 1), RealScalar.ZERO);
    Se2AnyEntity se2AnyEntity = new Se2AnyEntity(root, 8);
    se2AnyEntity.trajectoryPlannerCallback = OwlySimulation.trajectoryPlannerCallback;
    R2ImageRegionWrap r2ImageRegionWrap = R2ImageRegions._GTOB;
    ImageRegion imageRegion = r2ImageRegionWrap.imageRegion();
    imageRegion = r2ImageRegionWrap.imageRegion();
    se2AnyEntity.startLife(imageRegion, root); // (trq, root);
    OwlySimulation.set(se2AnyEntity);
    OwlySimulation.configCoordinateOffset(50, 700);
    OwlySimulation.addBackground(RegionRenders.create(imageRegion));
    OwlySimulation.jFrame.setBounds(100, 50, 800, 800);
    OwlySimulation.jFrame.setVisible(true);
    AnimationInterface controllable = se2AnyEntity;
    AbstractEntity abstractEntity = (AbstractEntity) controllable;
    List<TrajectorySample> head;
    AbstractAnyEntity abstractAnyEntity = (AbstractAnyEntity) abstractEntity;
    Tensor waypoints = Tensors.of( //
        Tensors.vector(5, 10, 1.5), //
        Tensors.vector(9.5, 9.5, 0), //
        Tensors.vector(9.5, 6.5, -1.5), //
        Tensors.vector(6.5, 6.5, -3)); //
    Tensor goal = waypoints.get(0);
    head = abstractEntity.getFutureTrajectoryUntil(abstractEntity.delayHint());
    abstractAnyEntity.switchToGoal(se2AnyEntity.trajectoryPlannerCallback, head, goal);
    int i = 0;
    while (true) {
      Tensor loc = abstractEntity.getEstimatedLocationAt(abstractEntity.delayHint());
      Scalar dist = se2AnyEntity.distance(loc, goal).abs();
      Scalar distThreshold = Scalars.fromString("2");
      if (Scalars.lessThan(dist, distThreshold)) { // if close enough to current waypoint switch to next
        i = (i + 1) % waypoints.length();
        goal = waypoints.get(i);
        head = abstractEntity.getFutureTrajectoryUntil(abstractEntity.delayHint());
        abstractAnyEntity.switchToGoal(se2AnyEntity.trajectoryPlannerCallback, head, goal);
      } else {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }

  public static void main(String[] args) throws Exception {
    new Se2GlcAnyTrackDemo().start();
  }
}
