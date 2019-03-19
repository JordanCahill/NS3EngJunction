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

int packetCount = 1;

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
//-- Callback function, called whenever a packet is received successfully. Prints stats to csv file.
//---------------------------------------------------------------------------------------
void ReceivePacket (Ptr<Node> node1, Ptr<Node> node2, Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while (packet = socket->Recv ())
    {

      SeqTsHeader seqTs;
      packet->RemoveHeader (seqTs);

      //---------------------------------------------------------------------------------------
      //-- Calculate end to end delay, transmission distance, and packet size
      //---------------------------------------------------------------------------------------
      int64_t now = Simulator::Now().GetNanoSeconds();
      int64_t txTime = seqTs.GetTs().GetNanoSeconds();
      int64_t delay = now - txTime;

      double distance = CalcNodeDistance(node1, node2);

      uint32_t size = packet->GetSize ();

      //---------------------------------------------------------------------------------------
      //-- Write results to CSV file, increment current packet counter
      //---------------------------------------------------------------------------------------
      std::ofstream datafile ("EngJuncData.csv", std::ios_base::app);
      if (datafile.is_open())
        {
          datafile << ::packetCount << ", " << now << ", " << delay << ", " << distance << ", " << size << "\n";
        }
      
      ++(::packetCount);
        
    }
}


//---------------------------------------------------------------------------------------
//-- Create Recursive Traffic Generator
//---------------------------------------------------------------------------------------
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {  
      Ptr<Packet> pkt = Create<Packet> (pktSize);
      SeqTsHeader hdr = SeqTsHeader();
      pkt->AddHeader(hdr); 
      socket->Send (pkt);

      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

//---------------------------------------------------------------------------------------
//-- Main Function
//---------------------------------------------------------------------------------------
int 
main (int argc, char *argv[])
{
  
  //-------------------------------------------------------------------------------------
  //-- Initialise Variables
  //-------------------------------------------------------------------------------------
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 100;
  double interval = 0.1; // seconds
  Time interPacketInterval = Seconds (interval);
  
  //-------------------------------------------------------------------------------------
  //-- Create Nodes
  //-------------------------------------------------------------------------------------
  NodeContainer sensorNodes;
  sensorNodes.Create (1);
  
  NodeContainer carNodes;
  carNodes.Create (1);

  //-------------------------------------------------------------------------------------
  //-- Set up the Wi-Fi NICs
  //-------------------------------------------------------------------------------------
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11); // ns-3 supports generate a pcap trace
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer sensorDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, sensorNodes);
  sensorDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, sensorNodes);
  NetDeviceContainer carDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, carNodes);
  carDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, carNodes);



  //---------------------------------------------------------------------------------------
  //-- Set position of Nodes
  //---------------------------------------------------------------------------------------
  // Fixed Node
  AnimationInterface::SetConstantPosition (sensorNodes.Get(0), 5, 14);
  
  // Car Node
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =  CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (1.5, 8.3, 0));
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (carNodes);

  //---------------------------------------------------------------------------------------
  //-- Set speed of car to 11.1111 m/s (= 40 km/hr)
  //---------------------------------------------------------------------------------------
  for (NodeContainer::Iterator i = carNodes.Begin (); i != carNodes.End (); ++i){
    Ptr<Node> node = (*i);
    node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(11.1111,0,0));
  }
 
  //---------------------------------------------------------------------------------------
  //-- Setup the Internet stack and assign IPV4 addresses
  //---------------------------------------------------------------------------------------
  InternetStackHelper internet;
  internet.Install (sensorNodes);
  internet.Install (carNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer sensorInterfaces = ipv4.Assign (sensorDevices);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0"); 
  Ipv4InterfaceContainer carInterfaces = ipv4.Assign (carDevices);

  //---------------------------------------------------------------------------------------
  //-- Setup socket connection and callback when packets are received by the source
  //---------------------------------------------------------------------------------------
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (carNodes.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeBoundCallback (&ReceivePacket, sensorNodes.Get (0), carNodes.Get (0)));

  //---------------------------------------------------------------------------------------
  //-- Begin broadcasting and generating traffic
  //---------------------------------------------------------------------------------------
  Ptr<Socket> source = Socket::CreateSocket (sensorNodes.Get (0), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);
  

  Simulator::Stop (Seconds (60));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
