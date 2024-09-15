#ifndef ANCHORED_SS_GAUSS_MARKOV_MOBILITY_MODEL_H
#define ANCHORED_SS_GAUSS_MARKOV_MOBILITY_MODEL_H

#include "constant-velocity-helper.h"
#include "mobility-model.h"
#include "position-allocator.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/box.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

class AnchoredSelfSimilarGaussMarkovMobilityModel : public MobilityModel
{
public:
  static TypeId GetTypeId (void);
  AnchoredSelfSimilarGaussMarkovMobilityModel ();

  // Functions to set parameters for randomness, decay factors, etc.
  void SetDecayFactors (double lambda1, double lambda2, double lambda3);
  void SetAlpha (double alpha);
  
protected:
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;

private:
  void Update (void);

  // Random variables for speed, direction, and pitch
  Ptr<RandomVariableStream> m_rndSpeed;
  Ptr<RandomVariableStream> m_rndDirection;
  Ptr<RandomVariableStream> m_rndPitch;
  Ptr<RandomVariableStream> m_meanVelocity;
  Ptr<RandomVariableStream> m_meanDirection;
  Ptr<RandomVariableStream> m_meanPitch;

  // Simulation parameters
  double m_alpha;
  double m_lambda1, m_lambda2, m_lambda3; // decay factors
  Vector m_velocity;
  Vector m_position;
  Time m_timeStep;  // TimeStep attribute
  Box m_bounds;     // Bounds attribute
};

} // namespace ns3

#endif // ANCHORED_SS_GAUSS_MARKOV_MOBILITY_MODEL_H
