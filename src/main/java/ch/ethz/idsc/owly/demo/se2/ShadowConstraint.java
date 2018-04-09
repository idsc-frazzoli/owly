// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owl.data.DontModify;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.core.Constraint;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.map.ShadowMap;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

@DontModify
public final class ShadowConstraint implements Constraint, Serializable {
  private final ShadowMap shadowMap;
  StateTime rootStateTime = null;

  public ShadowConstraint(ShadowMap shadowMap) {
    this.shadowMap = shadowMap;
  }

  @Override // from CostIncrementFunction
  public boolean isSatisfied(GlcNode glcNode, GlcNode parentNode, List<StateTime> trajectory) {
    //
    shadowMap.pause(); // TODO: hack, find cleaner solution to halt time or compensate for it
    // get time at root
    if (parentNode.isRoot()) {
      rootStateTime = parentNode.stateTime();
    }
    // check if shadowArea far enough away from node to be unreachable by shadow
    Tensor state = glcNode.state();
    double posX = state.Get(0).number().doubleValue();
    double posY = state.Get(1).number().doubleValue();
    double tDelt = glcNode.stateTime().time().number().doubleValue() //
        - rootStateTime.time().number().doubleValue();
    //
    double rad = tDelt * shadowMap.vMax;
    Shape circle = new Ellipse2D.Double(posX - rad, posY + rad, rad, rad);
    Area simShadowArea = (Area) shadowMap.getCurrentMap().clone();
    Area circleArea = new Area(circle);
    circleArea.intersect(simShadowArea);
    //
    if (circleArea.isEmpty())
      return true;
    //
    Scalar vel = glcNode.flow().getU().Get(0);
    Scalar tStop = vel.multiply(vel).multiply(DoubleScalar.of(2.0));
    // find node at t-tStop
    Scalar tMinTStop = (Scalar) glcNode.stateTime().time().subtract(tStop).unmodifiable();
    Scalar t = parentNode.stateTime().time();
    GlcNode targetNode = parentNode;
    //
    while (Scalars.lessThan(tMinTStop, t)) {
      if (targetNode.isRoot())
        break;
      targetNode = targetNode.parent();
      t = t.subtract(t.subtract(targetNode.stateTime().time()));
    }
    //
    List<StateTime> path = GlcNodes.getPathFromRootTo(targetNode);
    path.remove(0); // remove root
    if (!path.isEmpty()) {
      Scalar prevTime = rootStateTime.time();
      for (StateTime stateTime : path) {
        Scalar timeBetweenNodes = stateTime.time().subtract(prevTime);
        shadowMap.updateMap(simShadowArea, stateTime, timeBetweenNodes.number().floatValue());
        prevTime = stateTime.time();
      }
    }
    //
    // simulate shadow map until t with no new sensor info
    shadowMap.updateMap(simShadowArea, targetNode.stateTime(), tStop.number().floatValue());
    // check if node is inside simulated shadow area
    if (simShadowArea.contains(posX, posY)) {
      shadowMap.resume();
      return false;
    }
    //
    shadowMap.resume();
    return true;
  }
}
