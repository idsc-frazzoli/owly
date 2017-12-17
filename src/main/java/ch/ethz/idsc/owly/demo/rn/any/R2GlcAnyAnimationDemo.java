// code by jl and jph
package ch.ethz.idsc.owly.demo.rn.any;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class R2GlcAnyAnimationDemo implements DemoInterface {
  ;
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    StateTime root = new StateTime(Tensors.vector(0.2, 0.2), RealScalar.ZERO);
    R2AnyEntity r2AnyEntity = new R2AnyEntity(root, 18);
    r2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    Region<Tensor> obstacleRegion = new R2NoiseRegion(RealScalar.of(0.1));
    r2AnyEntity.startLife(obstacleRegion, root); // (trq, root);
    TrajectoryRegionQuery trajectoryRegionQuery = SimpleTrajectoryRegionQuery.timeInvariant(obstacleRegion);
    owlyAnimationFrame.set(r2AnyEntity);
    owlyAnimationFrame.setObstacleQuery(trajectoryRegionQuery);
    owlyAnimationFrame.addBackground(RegionRenders.create(trajectoryRegionQuery));
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2GlcAnyAnimationDemo().start();
  }
}
