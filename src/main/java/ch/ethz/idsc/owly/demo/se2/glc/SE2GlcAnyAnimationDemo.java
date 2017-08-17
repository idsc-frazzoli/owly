// code by jl
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.demo.se2.any.SE2AnyEntity;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum SE2GlcAnyAnimationDemo {
  ;
  public static void main(String[] args) throws Exception {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(2.5, 0.75, 0);
    SE2AnyEntity SE2AnyEntity = new SE2AnyEntity(root, 12);
    // TODO not final solution
    SE2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    // Region obstacleRegion = new InvertedRegion(EmptyRegion.INSTANCE);
    ImageRegion imageRegion = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(10, 10), false);
    // Region obstacleRegion = new R2NoiseRegion(0.8);
    SE2AnyEntity.startLife(imageRegion, root); // (trq, root);
    owlyAnimationFrame.set(SE2AnyEntity);
    // owlyAnimationFrame.setObstacleQuery(trq); //TODO JAN: Why external connection needed?
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.addBackground(imageRegion);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
