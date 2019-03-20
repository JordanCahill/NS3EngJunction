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

//int packetCount = 1;
int totalDelay = 0;

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

      ::totalDelay += delay;
        
    }
}


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
  uint32_t totalData = 15000; // bytes
  uint32_t maxPacketSize = 1500; // bytes - MTU for IPv6 over 802.11p = 1500
  double t_interval = 0.1; // seconds
  Time interPacketInterval = Seconds (t_interval);

  CommandLine cmd;
  cmd.AddValue("totalData", "Total Data to transmit (in bytes)", totalData);
  cmd.Parse(argc, argv);
  
  //-------------------------------------------------------------------------------------
  //-- Create Nodes
  //-------------------------------------------------------------------------------------
  NodeContainer sensorNodes;
  sensorNodes.Create (2);
  
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


  //---------------------------------------------------------------------------------------
  //-- Set position of Nodes (10m apart)
  //---------------------------------------------------------------------------------------

  AnimationInterface::SetConstantPosition (sensorNodes.Get(0), 5, 10);
  AnimationInterface::SetConstantPosition (sensorNodes.Get(1), 5, 20);
 
  //---------------------------------------------------------------------------------------
  //-- Setup the Internet stack and assign IPV4 addresses
  //---------------------------------------------------------------------------------------
  InternetStackHelper internet;
  internet.Install (sensorNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer sensorInterfaces = ipv4.Assign (sensorDevices);

  //---------------------------------------------------------------------------------------
  //-- Setup socket connection and callback when packets are received by the source
  //---------------------------------------------------------------------------------------
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (sensorNodes.Get (1), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeBoundCallback (&ReceivePacket, sensorNodes.Get (0), sensorNodes.Get (1)));

  //---------------------------------------------------------------------------------------
  //-- Begin broadcasting and generating traffic
  //---------------------------------------------------------------------------------------
  Ptr<Socket> source = Socket::CreateSocket (sensorNodes.Get (0), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  AnimationInterface anim ("test.xml");

  //---------------------------------------------------------------------------------------
  //-- Calc number of packets needed, if there's a remainder since the datatypes are ints,
  //-- the program will round down, this is accounted for in the if loop
  //---------------------------------------------------------------------------------------
  uint32_t numPackets = totalData/maxPacketSize;
  if (std::abs(std::remainder(totalData, maxPacketSize)) != 0)
   {
     ++numPackets;
   }

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (2), &GenerateTraffic,
                                  source, maxPacketSize, numPackets, 
                                  interPacketInterval, totalData, totalData);

  Simulator::Stop (Seconds (600));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}

