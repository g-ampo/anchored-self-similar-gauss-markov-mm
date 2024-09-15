
#include "anchored-ss-gauss-markov-mobility-model.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include <cmath>
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "position-allocator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("AnchoredSelfSimilarGaussMarkovMobilityModel");

TypeId
AnchoredSelfSimilarGaussMarkovMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AnchoredSelfSimilarGaussMarkovMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<AnchoredSelfSimilarGaussMarkovMobilityModel> ()
    .AddAttribute ("Bounds",
                   "The 3D bounding area for the mobility model.",
                   BoxValue (Box (0.0, 500.0, 0.0, 500.0, 0.0, 300.0)),  // Default bounds
                   MakeBoxAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_bounds),
                   MakeBoxChecker ())
    .AddAttribute ("TimeStep",
/**
 * \brief Default constructor.
 *
 * Sets default values for decay factors (0.85) and schedules
 * the first update for 0.5 seconds in the future.
 */
                   "Time step for mobility updates.",
                   TimeValue (Seconds (0.5)),
                   MakeTimeAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_timeStep),
                   MakeTimeChecker ())
    .AddAttribute ("Alpha",
                   "Randomness parameter.",
                   DoubleValue (0.85),
                   MakeDoubleAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_alpha),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MeanVelocity",
                   "Mean velocity of the node.",
                   StringValue ("ns3::UniformRandomVariable[Min=10|Max=20]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_meanVelocity),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("MeanDirection",
                   "Mean direction of the node.",
                   StringValue ("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_meanDirection),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("MeanPitch",
                   "Mean pitch of the node.",
                   StringValue ("ns3::UniformRandomVariable[Min=-0.1|Max=0.1]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_meanPitch),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("NormalVelocity",
                   "Randomness for velocity.",
                   StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_rndSpeed),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("NormalDirection",
                   "Randomness for direction.",
                   StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_rndDirection),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("NormalPitch",
                   "Randomness for pitch.",
                   StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02]"),
                   MakePointerAccessor (&AnchoredSelfSimilarGaussMarkovMobilityModel::m_rndPitch),
                   MakePointerChecker<RandomVariableStream> ());
  return tid;
}


NS_OBJECT_ENSURE_REGISTERED (AnchoredSelfSimilarGaussMarkovMobilityModel);

AnchoredSelfSimilarGaussMarkovMobilityModel::AnchoredSelfSimilarGaussMarkovMobilityModel ()
{
  m_alpha = 0.85;
  m_lambda1 = m_lambda2 = m_lambda3 = 1.0; // Default decay factors
  m_rndSpeed = CreateObject<NormalRandomVariable> ();
  m_rndDirection = CreateObject<NormalRandomVariable> ();
  m_rndPitch = CreateObject<NormalRandomVariable> ();

  Simulator::Schedule (Seconds (0.5), &AnchoredSelfSimilarGaussMarkovMobilityModel::Update, this);
}

void
AnchoredSelfSimilarGaussMarkovMobilityModel::SetDecayFactors (double lambda1, double lambda2, double lambda3)
{
  m_lambda1 = lambda1;
  m_lambda2 = lambda2;
  m_lambda3 = lambda3;
}

void
AnchoredSelfSimilarGaussMarkovMobilityModel::SetAlpha (double alpha)
{
  m_alpha = alpha;
}

void
AnchoredSelfSimilarGaussMarkovMobilityModel::DoInitialize (void)
{
  m_velocity = Vector (0.0, 0.0, 0.0);
  m_position = Vector (0.0, 0.0, 0.0);
  MobilityModel::DoInitialize ();
}

Vector
AnchoredSelfSimilarGaussMarkovMobilityModel::DoGetPosition (void) const
{
  return m_position;
}

void
AnchoredSelfSimilarGaussMarkovMobilityModel::DoSetPosition (const Vector &position)
{
  m_position = position;
}

Vector
AnchoredSelfSimilarGaussMarkovMobilityModel::DoGetVelocity (void) const
{
  return m_velocity;
}

void
AnchoredSelfSimilarGaussMarkovMobilityModel::Update (void)
{
  // Get the previous values for speed, direction, and pitch
  double prevSpeed = m_velocity.x;
  double prevDirection = m_velocity.y;
  double prevPitch = m_velocity.z;

  // Update randomness indices
  double a_s = m_alpha * exp(-1.0 / m_lambda1 * fabs(prevSpeed - m_velocity.x));
  double a_d = m_alpha * exp(-1.0 / m_lambda2 * fabs(prevDirection - m_velocity.y));
  double a_p = m_alpha * exp(-1.0 / m_lambda3 * fabs(prevPitch - m_velocity.z));

  // Calculate new velocity components
  double newSpeed = a_s * prevSpeed + (1 - a_s) * m_rndSpeed->GetValue ();
  double newDirection = a_d * prevDirection + (1 - a_d) * m_rndDirection->GetValue ();
  double newPitch = a_p * prevPitch + (1 - a_p) * m_rndPitch->GetValue ();

  m_velocity = Vector (newSpeed, newDirection, newPitch);

  // Update position
  m_position.x += newSpeed * cos (newDirection) * cos (newPitch);
  m_position.y += newSpeed * sin (newDirection) * cos (newPitch);
  m_position.z += newSpeed * sin (newPitch);

  // Get the node's ID and print it with the updated position and velocity
  Ptr<Node> node = GetObject<Node>();
  uint32_t nodeId = node->GetId();  // Get the node ID

  // Print the updated position and velocity for debugging
  std::cout << "Node ID: " << nodeId << std::endl;
  std::cout << "Position: (" << m_position.x << ", " << m_position.y << ", " << m_position.z << ")" << std::endl;
  std::cout << "Velocity: (" << newSpeed << ", " << newDirection << ", " << newPitch << ")" << std::endl;

  // Schedule the next update
  Simulator::Schedule (Seconds (0.5), &AnchoredSelfSimilarGaussMarkovMobilityModel::Update, this);
}

} // namespace ns3
