// code by jph
package ch.ethz.idsc.owly.math;

enum StaticHelper {
  ;
  static StateTimeTensorFunction STATE_TIME = //
      stateTime -> stateTime.state().copy().append(stateTime.time());
}
