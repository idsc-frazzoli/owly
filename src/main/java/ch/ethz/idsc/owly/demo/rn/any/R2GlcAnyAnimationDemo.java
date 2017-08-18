// code by jl and jph
package ch.ethz.idsc.owly.demo.rn.any;

import ch.ethz.idsc.owly.demo.rn.R2AnyEntity;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2GlcAnyAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(0.2, 0.2);
    R2AnyEntity r2AnyEntity = new R2AnyEntity(root, 15);
    // TODO not final solution
    r2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    Region obstacleRegion = new R2NoiseRegion(0.1);
    // TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery(obstacleRegion);
    r2AnyEntity.startLife(obstacleRegion, root); // (trq, root);
    owlyAnimationFrame.set(r2AnyEntity);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
