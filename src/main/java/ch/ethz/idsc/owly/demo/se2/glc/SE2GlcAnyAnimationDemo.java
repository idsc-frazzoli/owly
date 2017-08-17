// code by jl
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.se2.any.SE2AnyEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum SE2GlcAnyAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(0.2, 0.2, 0);
    SE2AnyEntity SE2AnyEntity = new SE2AnyEntity(root);
    // TODO not final solution
    SE2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    Region obstacleRegion = new InvertedRegion(EmptyRegion.INSTANCE);
//    Region obstacleRegion = new R2NoiseRegion(0.05);
    // TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery(obstacleRegion);
    SE2AnyEntity.startLife(obstacleRegion, root); // (trq, root);
    owlyAnimationFrame.set(SE2AnyEntity);
    // owlyAnimationFrame.setObstacleQuery(trq); //TODO JAN: Why external connection needed?
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
