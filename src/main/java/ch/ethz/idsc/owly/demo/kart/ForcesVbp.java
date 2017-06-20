// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.Scalar;

class ForcesVbp {
  Scalar f_FLX;
  Scalar f_FLY;
  Scalar f_FRX;
  Scalar f_FRY;
  Scalar f_RLX;
  Scalar f_RLY;
  Scalar f_RRX;
  Scalar f_RRY;

  static ForcesVbp compute(Scalar h, KartState status, Scalar delta, Scalar timeStep) {
    MuScriptVbp s = new MuScriptVbp();
    MuStruct muStruct = s.friction(status, delta);
    WeightTransfer wt = new WeightTransfer(); // FIXME
    ForcesZ forcesZ = wt.compute(h, status, delta, timeStep); // FIXME
    // ---
    ForcesVbp forcesVbp = new ForcesVbp();
    forcesVbp.f_FLX = muStruct.mu_FLX.multiply(forcesZ.f_FLZ);
    forcesVbp.f_FLY = muStruct.mu_FLY.multiply(forcesZ.f_FLZ);
    //
    forcesVbp.f_FRX = muStruct.mu_FRX.multiply(forcesZ.f_FRZ);
    forcesVbp.f_FRY = muStruct.mu_FRY.multiply(forcesZ.f_FRZ);
    //
    forcesVbp.f_RLX = muStruct.mu_RLX.multiply(forcesZ.f_RLZ);
    forcesVbp.f_RLY = muStruct.mu_RLY.multiply(forcesZ.f_RLZ);
    //
    forcesVbp.f_RRX = muStruct.mu_RRX.multiply(forcesZ.f_RRZ);
    forcesVbp.f_RRY = muStruct.mu_RRY.multiply(forcesZ.f_RRZ);
    return forcesVbp;
  }

  Scalar f_FX() {
    return f_FLX.add(f_FRX);
  }

  Scalar f_RX() {
    return f_RLX.add(f_RRX);
  }

  Scalar f_FY() {
    return f_FLY.add(f_FRY);
  }

  Scalar f_RY() {
    return f_RLY.add(f_RRY);
  }
}
