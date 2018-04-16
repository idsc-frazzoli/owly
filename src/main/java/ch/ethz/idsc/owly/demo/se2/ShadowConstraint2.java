// code by ynager
package ch.ethz.idsc.owly.demo.se2;

import java.awt.BasicStroke;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.io.Serializable;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ch.ethz.idsc.owl.data.DontModify;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.core.Constraint;
import ch.ethz.idsc.owl.glc.core.DomainQueue;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.map.ShadowMap;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Entity;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sin;

@DontModify
public final class ShadowConstraint2 implements Constraint, Serializable {
  private final ShadowMap shadowMap;
  // FIXME design error prone because calling isSatisfied(...) alters state of instance
  // ... that means the order in which to call isSatisfied determines the outcome of the test
  private final Scalar gamma;
  private final Scalar reactionTime = DoubleScalar.of(0.5);
  private StateTime rootStateTime = null;
  private final Tensor eta;
  final Map<Tensor, Area> map = new HashMap<>();
  long counter = 0;
  long total = 0;

  public ShadowConstraint2(ShadowMap shadowMap, Scalar gamma,  Tensor eta) {

    this.shadowMap = shadowMap;
    this.gamma = gamma;
    this.eta = eta;
  }
  
  protected Tensor convertToKey(StateTime stateTime) {
    return eta.pmul(stateTime.state()).map(Floor.FUNCTION);
  }

  @Override // from CostIncrementFunction
  public boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    //
    long startTime = System.nanoTime();
    
    shadowMap.pause(); // TODO: hack, find cleaner solution to halt time or compensate for it
    // get time at root
    if (glcNode.isRoot()) {
      rootStateTime = glcNode.stateTime();
      map.put(convertToKey(rootStateTime), (Area) shadowMap.getCurrentMap().clone());
    }
    
    StateTime childStateTime = trajectory.get(trajectory.size()-1);
    double posX = childStateTime.state().Get(0).number().doubleValue();
    double posY = childStateTime.state().Get(1).number().doubleValue();

    Scalar vel = flow.getU().Get(0);
    Scalar tStop = vel.multiply(vel).multiply(gamma).add(reactionTime);
    Scalar dStop = vel.multiply(tStop).divide(RealScalar.of(2));
    
    // simulate shadowmap from node to child
    Area simShadowArea = (Area) map.get(convertToKey(glcNode.stateTime())).clone();
    //Scalar prevTime = glcNode.stateTime().time();
    //Scalar timeBetweenNodes = childStateTime.time().subtract(prevTime);
    shadowMap.updateMap(simShadowArea, childStateTime, 0.4f);
    map.put(convertToKey(childStateTime), (Area) simShadowArea.clone());
    
    // simulate shadow map during braking
    shadowMap.updateMap(simShadowArea, childStateTime, tStop.number().floatValue());
    
    // calculate braking path
    Scalar angle = childStateTime.state().Get(2);
    double deltaX = Cos.of(angle).multiply(dStop).number().doubleValue();
    double deltaY = Sin.of(angle).multiply(dStop).number().doubleValue();
    Line2D brakingLine = new Line2D.Double(posX, posY, posX+deltaX, posY+deltaY);
    //System.out.print("x1: " + posX  +" x2: "+ x2 + " y1: "+ posY + " y2: " + y2 + "\n");
    Stroke stroke = new BasicStroke(0.001f, BasicStroke.CAP_ROUND, BasicStroke.CAP_BUTT);
    Area brakingLineArea = new Area(stroke.createStrokedShape(brakingLine));  
    simShadowArea.intersect(brakingLineArea);
    boolean value = simShadowArea.isEmpty();
    //boolean value = !simShadowArea.contains(posX+deltaX, posY+deltaY);
    shadowMap.resume();
    
    //long stopTime = System.nanoTime();
    //long duration = stopTime - startTime;
    
    //total = total + duration;
    //System.out.print("mean duration: " + total/(counter+1) + "     counter: " + counter + "\n");
    //counter += 1;
    
    return value;
  }
}
