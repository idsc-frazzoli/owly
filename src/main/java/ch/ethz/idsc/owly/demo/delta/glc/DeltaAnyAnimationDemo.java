// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class DeltaAnyAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    int resolution = 9;
    List<Tensor> obstacleInitList = new ArrayList<>();
    obstacleInitList.add(Tensors.vector(2, 0.35, 0.2));
    obstacleInitList.add(Tensors.vector(3, 5.2, 0.35));
    obstacleInitList.add(Tensors.vector(1.5, 1, 0.4));
    obstacleInitList.add(Tensors.vector(5.3, 3.2, 0.25));
    // ---
    ImageRegion imageRegion = new ImageRegion(DeltaAnyEntity.IMAGE_OBSTACLE, DeltaAnyEntity.RANGE, true);
    // --
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    DeltaAnyEntity entity = new DeltaAnyEntity(obstacleInitList, new StateTime(Tensors.vector(8.8, 0.5), RealScalar.ZERO), resolution);
    entity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    entity.startLife(imageRegion, new StateTime(Tensors.vector(8.8, 0.5), RealScalar.ZERO));
    // ---
    // ---
    owlyAnimationFrame.set(entity);
    owlyAnimationFrame.setObstacleQuery(entity.getTrajectoryRegionQuery());
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    for (int i = 0; i < obstacleInitList.size(); i++) {
      owlyAnimationFrame.addBackground((RenderInterface) entity.getFloatingObstacle(i));
    }
    owlyAnimationFrame.jFrame.setVisible(true);
    owlyAnimationFrame.configCoordinateOffset(33, 416);
    owlyAnimationFrame.jFrame.setBounds(100, 100, 620, 475);
  }

  public static void main(String[] args) throws Exception {
    new DeltaAnyAnimationDemo().start();
  }
}
