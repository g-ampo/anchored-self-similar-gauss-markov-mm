#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/anchored-ss-gauss-markov-mobility-model.h"  // Include the custom model

using namespace ns3;

int main (int argc, char *argv[])
{
  // Initialize NS3 simulation
  CommandLine cmd;
  cmd.Parse (argc, argv);

  // Create nodes for the drone swarm
  NodeContainer droneSwarm;
  droneSwarm.Create (20); // Example with 20 drones

  // Mobility Model - Anchored Self-Similar 3D Gauss-Markov
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::AnchoredSelfSimilarGaussMarkovMobilityModel", // Use the custom model
                             "Bounds", BoxValue (Box (0, 500, 0, 500, 0, 300)),
                             "TimeStep", TimeValue (Seconds (0.5)),
                             "Alpha", DoubleValue (0.85),
                             "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=10|Max=20]"),
                             "MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                             "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=-0.1|Max=0.1]"),
                             "NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0]"),
                             "NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1]"),
                             "NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02]"));
  
  // Set positions
  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                                 "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=500]"),
                                 "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=500]"),
                                 "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=300]"));

  // Install the mobility model on all drones
  mobility.Install (droneSwarm);

  // Internet Stack to enable communication
  InternetStackHelper internet;
  internet.Install (droneSwarm);

  // Set up simple communication scenario, e.g., UDP echo

  // Run simulation
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
