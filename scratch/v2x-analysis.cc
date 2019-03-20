/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <iostream>
#include <fstream>

using namespace std;

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/netanim-module.h"

#include "ns3/flow-monitor-module.h"
#include "ns3/seq-ts-header.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

using namespace ns3;

int totalDelay = 0;

AnimationInterface * anim = 0;

NS_LOG_COMPONENT_DEFINE ("v2x-analysis");  // Allow logging

//---------------------------------------------------------------------------------------
//-- Logs any change in a node's course
//---------------------------------------------------------------------------------------
void
CourseChange (std:: string context, Ptr<const MobilityModel> model)
{
  Vector position = model->GetPosition ();
  NS_LOG_UNCOND (context <<
    " x = " << position.x << ", y = " << position.y);
}

//---------------------------------------------------------------------------------------
//-- Calculates and returns the straight line distance (m) between two nodes and
//-- returns as a double
//---------------------------------------------------------------------------------------
double
CalcNodeDistance(Ptr<Node> node1, Ptr<Node> node2)
{
  Ptr<MobilityModel> model1 = node1->GetObject<MobilityModel>();
  Ptr<MobilityModel> model2 = node2->GetObject<MobilityModel>();
  double distance = model1->GetDistanceFrom (model2);
  return distance;
}

//---------------------------------------------------------------------------------------
//-- Callback function is called whenever a packet is received successfully.
//---------------------------------------------------------------------------------------
void ReceivePacket (Ptr<Node> node1, Ptr<Node> node2, Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while (packet = socket->Recv ())
    {

      SeqTsHeader seqTs;
      packet->RemoveHeader (seqTs);

      //---------------------------------------------------------------------------------------
      //-- Calculate delay, distance between nodes, and packet size
      //---------------------------------------------------------------------------------------
      int64_t now = Simulator::Now().GetNanoSeconds();
      int64_t txTime = seqTs.GetTs().GetNanoSeconds();
      int64_t delay = now - txTime;
      ::totalDelay += delay;

      double distance = CalcNodeDistance(node1, node2);

      uint32_t size = packet->GetSize ();

      //---------------------------------------------------------------------------------------
      //-- Log data
      //---------------------------------------------------------------------------------------
      std::ofstream datafile ("v2x_analysis_log.txt", std::ios_base::app);
      if (datafile.is_open())
        {
          datafile << "Packet received at time: " << now << " ns, Tx delay: " << delay << " ns, Tx distance: " << distance << " m, Packet Size: " << size << " bytes\n";
        }       
    }
}


//---------------------------------------------------------------------------------------
//-- Currently not used, needs to be updated
//---------------------------------------------------------------------------------------
/*
void HideNode (Ptr<Node> node0)
{
  uint32_t id = node0 ->GetId ();
  anim->UpdateNodeSize (id, 0.01, 0.01);
  anim->UpdateNodeDescription (node0, "");
  Ptr<ConstantVelocityMobilityModel> mob = node0->GetObject<ConstantVelocityMobilityModel>();
  mob->SetVelocity(Vector(0, 0, 0)); 

  Ptr<Ipv4> ipv4 = node0->GetObject<Ipv4> ();
  //Ptr<NetDevice> device = node0->GetDevice(0);
  //int32_t interface = ipv4->GetInterfaceForDevice(device);
  //int32_t interface = ipv4->GetInterfaceForAddress("10.1.2.0");
  //std::cout << interface << "\n";
  ipv4->SetDown (1);
}
*/

//---------------------------------------------------------------------------------------
//-- Currently not used, needs to be updated
//---------------------------------------------------------------------------------------
/*
void TurnLeft (Ptr<Node> node0)
{
  Ptr<MobilityModel> mobility = node0 ->GetObject<MobilityModel>();
  Vector current = mobility->GetPosition();
  if (current.x > 8.2)
  {
     Ptr<ConstantVelocityMobilityModel> mob = node0->GetObject<ConstantVelocityMobilityModel>();
     mob->SetVelocity(Vector(8, -40, 0));    
  }

  if (Simulator::Now ().GetSeconds () < 0.7)
  {
     Simulator::Schedule (Seconds (0.05), TurnLeft, node0);
  }
}
*/

//---------------------------------------------------------------------------------------
//-- Create Recursive Traffic Generator
//---------------------------------------------------------------------------------------
static void GenerateTraffic (Ptr<Socket> socket, uint32_t maxPktSize,
                             uint32_t pktCount, Time pktInterval, 
                             uint32_t totalData, uint32_t dataLeft)
{
  //---------------------------------------------------------------------------------------
  //-- If packets left, send
  //---------------------------------------------------------------------------------------
  uint32_t pktSize = maxPktSize;
  if (pktCount > 0)
    {  
      if (dataLeft < maxPktSize)
        {
          pktSize = dataLeft;
        }
      Ptr<Packet> pkt = Create<Packet> (pktSize);
      SeqTsHeader hdr = SeqTsHeader();
      pkt->AddHeader(hdr); 
      socket->Send (pkt);
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, maxPktSize, pktCount - 1, pktInterval, totalData, dataLeft - pktSize);
    }
  //---------------------------------------------------------------------------------------
  //-- If no packets left, log data and close socket
  //---------------------------------------------------------------------------------------
  else
    {
      std::ofstream datafile ("EngJuncSize.csv", std::ios_base::app);
      if (datafile.is_open())
        {
          datafile << totalData << ", " << ::totalDelay << "\n";
        }
      socket->Close ();
    }
}

int 
main (int argc, char *argv[])
{
  //-------------------------------------------------------------------------------------
  //-- Initialise Variables
  //-------------------------------------------------------------------------------------
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t totalData = 15000; // bytes
  uint32_t maxPacketSize = 1500; // bytes - MTU for IPv6 over 802.11p = 1500
  uint32_t numSensorNodes = 2;
  uint32_t numCarNodes = 1;
  double interval = 0.1; // seconds
  Time interPacketInterval = Seconds (interval);
  
  //-------------------------------------------------------------------------------------
  //-- Add options to change variables from the command line
  //-------------------------------------------------------------------------------------
  CommandLine cmd;
  cmd.AddValue("totalData", "Total Data to transmit (bytes)", totalData);
  cmd.AddValue("numCarNodes", "Number of car nodes", numCarNodes);
  cmd.AddValue("numSensorNodes", "Number of roadside sensor nodes", numSensorNodes);
  cmd.AddValue("maxPacketSize", "MTU of protocol (bytes)", maxPacketSize);
  cmd.Parse(argc, argv);

  //-------------------------------------------------------------------------------------
  //-- Create Nodes
  //-------------------------------------------------------------------------------------
  NodeContainer sensorNodes;
  sensorNodes.Create (numSensorNodes);
  
  NodeContainer carNodes;
  carNodes.Create (numCarNodes);

  //-------------------------------------------------------------------------------------
  //-- Set up the Wi-Fi NICs
  //-------------------------------------------------------------------------------------
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11); // ns-3 supports pcap tracing
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer sensorDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, sensorNodes);
  sensorDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, sensorNodes);
  NetDeviceContainer carDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, carNodes);
  carDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, carNodes);
  
  //-------------------------------------------------------------------------------------
  //-- Set position of each node - TODO: Create function to do this
  //-------------------------------------------------------------------------------------
  AnimationInterface::SetConstantPosition (sensorNodes.Get(0), 15, 5.5);
  AnimationInterface::SetConstantPosition (sensorNodes.Get(1), 5, 14);


  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =  CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (1.5, 8.3, 0));
  positionAlloc->Add (Vector (0, 1, 0));  
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (carNodes);

  //---------------------------------------------------------------------------------------
  //-- Set speed to a random value between 5 and 10 m/s (18 and 36 km/hr respectively
  //---------------------------------------------------------------------------------------
  Ptr<UniformRandomVariable> rvar = CreateObject<UniformRandomVariable>();
  for (NodeContainer::Iterator i = carNodes.Begin (); i != carNodes.End (); ++i){
    Ptr<Node> node = (*i);
    double speed = rvar->GetValue(5, 10);
    node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed,0,0));
  }
  
  //---------------------------------------------------------------------------------------
  //-- Setup the Internet stack and assign IPV4 addresses
  //---------------------------------------------------------------------------------------
  InternetStackHelper internet;
  internet.Install (sensorNodes);
  internet.Install (carNodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer sensorInterfaces = ipv4.Assign (sensorDevices);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0"); // Should the cars and rsu be of the same base address?
  Ipv4InterfaceContainer carInterfaces = ipv4.Assign (carDevices);

  //---------------------------------------------------------------------------------------
  //-- Setup socket connection and callback when packets are received by the source
  //---------------------------------------------------------------------------------------
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (carNodes.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeBoundCallback (&ReceivePacket, sensorNodes.Get (1), carNodes.Get (0)));

  Ptr<Socket> source = Socket::CreateSocket (sensorNodes.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  //---------------------------------------------------------------------------------------
  //-- Calc number of packets needed, if there's a remainder since the datatypes are ints,
  //-- the program will round down, this is accounted for in the if loop
  //---------------------------------------------------------------------------------------
  uint32_t numPackets = totalData/maxPacketSize;
  if (std::abs(std::remainder(totalData, maxPacketSize)) != 0)
   {
     ++numPackets;
   }

  //---------------------------------------------------------------------------------------
  //-- Begin generating traffic
  //---------------------------------------------------------------------------------------
  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (2), &GenerateTraffic,
                                  source, maxPacketSize, numPackets, 
                                  interPacketInterval, totalData, totalData);

  //TODO: Edit current "turn left" function to produce random turns
  

  //---------------------------------------------------------------------------------------
  //-- Apply netanim tracing
  //---------------------------------------------------------------------------------------
  anim = new AnimationInterface ("engjunction.xml");

  anim->SetBackgroundImage ("/home/jordan/Pictures/ns3/engjunctionlowres.png", 0, 0, 0.05, 0.05, 0.8);
  uint32_t carImageID = anim->AddResource ("/home/jordan/Pictures/ns3/car.png");
  uint32_t rsuImageID = anim->AddResource ("/home/jordan/Pictures/ns3/node.png");

  for (uint32_t i = 0; i < carNodes.GetN (); ++i)
    {
      uint32_t nodeID = carNodes.Get (i)->GetId ();
      anim->UpdateNodeDescription (carNodes.Get (i), "Car");
      anim->UpdateNodeColor (carNodes.Get (i), 0, 0, 255);
      anim->UpdateNodeImage (nodeID, carImageID);
      anim->UpdateNodeSize (nodeID, 2, 5);   
    }

  for (uint32_t i = 0; i < sensorNodes.GetN (); ++i)
    {
      uint32_t nodeID = sensorNodes.Get (i)->GetId ();
      anim->UpdateNodeDescription (sensorNodes.Get (i), "RSU");
      anim->UpdateNodeColor (sensorNodes.Get (i), 0, 255, 0);  
      anim->UpdateNodeImage (nodeID, rsuImageID);   
      anim->UpdateNodeSize (nodeID, 2, 2);  
    }
  
  //---------------------------------------------------------------------------------------
  //-- Apply flowmonitor tracing, currently not working
  //---------------------------------------------------------------------------------------
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.InstallAll();
  
  Simulator::Stop (Seconds (60));

  Simulator::Run ();
  flowMonitor->SerializeToXmlFile("EngJuncFM.xml", true, true);
  Simulator::Destroy ();
  return 0;
}
