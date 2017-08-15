// code by jl and jph
package ch.ethz.idsc.owly.demo.rn.any;

import ch.ethz.idsc.owly.demo.rn.R2AnyEntity;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2GlcAnyAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(0.2, 0.2);
    R2AnyEntity r2AnyEntity = new R2AnyEntity(root);
    // TODO not final solution
    r2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    Region region = new R2NoiseRegion(.2);
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
    r2AnyEntity.startLife(trq, root);
    owlyAnimationFrame.set(r2AnyEntity);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
