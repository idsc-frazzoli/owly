// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.data.DontModify;
import ch.ethz.idsc.owl.glc.core.CostFunction;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.map.ShadowMap;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;
import ch.ethz.idsc.tensor.sca.Sign;

/** cost function that penalizes the switching between forwards and backwards driving */
// DO NOT MODIFY THIS CLASS SINCE THE FUNCTIONALITY IS USED IN MANY DEMOS
@DontModify
public final class ShadowCostFunction implements CostFunction {

  ShadowMap shadowMap;
  StateTime rootStateTime;

  public ShadowCostFunction(ShadowMap shadowMap) {
    this.shadowMap = shadowMap;
    

  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    
    // calculate time needed to stop
    if (glcNode.isRoot()) {
      this.rootStateTime = glcNode.stateTime();
      return RealScalar.of(0);
    }
    
    //check if shadowArea far enough away from node to be unreachable by shadow
    double posX = glcNode.state().Get(0).number().doubleValue();
    double posY = glcNode.state().Get(1).number().doubleValue();
    double tDelt = glcNode.stateTime().time().number().doubleValue() - rootStateTime.time().number().doubleValue();
    
    double rad = tDelt*shadowMap.vMax;
    Shape circle = new Ellipse2D.Double(posX-rad, posY+rad, rad, rad);
    Area simShadowArea = (Area) shadowMap.getCurrentMap().clone();
    
    Area circleArea = new Area(circle);
    circleArea.intersect(simShadowArea);

    if(circleArea.isEmpty()) {
      return RealScalar.of(0);
    }
    
    Scalar vel = glcNode.flow().getU().Get(0);
    Scalar tStop = vel.multiply(vel).multiply(DoubleScalar.of(2.5));
    
    // find node at t-tStop
    Scalar tMinTStop = (Scalar) glcNode.stateTime().time().subtract(tStop).unmodifiable();
    Scalar t = glcNode.stateTime().time();
    GlcNode targetNode = glcNode;
    
    while (Scalars.lessThan(tMinTStop, t)) {
      if(targetNode.isRoot())
        break;
      targetNode = targetNode.parent();
      t = t.subtract(t.subtract(targetNode.stateTime().time()));
    }
        
    //get root node and build node list
    List<GlcNode> nodeList = new ArrayList<GlcNode>();
    
    GlcNode nodeIt = targetNode;
    while(!nodeIt.isRoot()) {
      nodeList.add(nodeIt);
      nodeIt = nodeIt.parent();
    }
    Collections.reverse(nodeList);  //reverse list, now starting at root
    
    //get current shadowMap 
    
    //simulate shadow map evolution to targetNode
    if(!nodeList.isEmpty())
    {
      for (GlcNode node : nodeList) {
        Scalar timeDelta = node.stateTime().time().subtract(node.parent().stateTime().time());
        shadowMap.updateMap(simShadowArea, node.stateTime(), timeDelta.number().floatValue());
      }
    }
    
    //simulate shadow map until t with no new sensor info
    shadowMap.updateMap(simShadowArea,  targetNode.stateTime(),tStop.number().floatValue());
    
    if(simShadowArea.contains(posX,posY)) {
      shadowMap.resume();
      return RealScalar.of(100000000);
    }
    
    shadowMap.resume();
    return RealScalar.of(0);
    
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }
}
