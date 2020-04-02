#include <cstdlib>
#include <iostream>
#include <string>
#include <memory>
#include <fstream>
#include <random>

#include "ns3/core-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/lte-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/network-module.h"
#include <ns3/buildings-helper.h>
#include "ns3/netanim-module.h"
#include "ns3/stats-module.h"
#include "ns3/evalvid-client-server-helper.h"
#include "ns3/evalvid-client.h"
#include "ns3/evalvid-server.h"

#define LOG(x) std::cout << x << std::endl
#define wait std::cin.get()

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteEvalvid");

/*
NOTAS SIMULAÇAO
remote host para receber video
usuarios moveis
drones
mover os drones p proximo das pessoas e enviar para algum local (link confiável?)
pessoas distribuidas aleatoriamente (qual distrib?)
clusterizar no python e mandar drone p centroide
time to cluster
time ro reach local
video quality
energy use
*/

std::string exec(const char* cmd);

void print_position(NodeContainer ueNodes)
{
    std::cout << '\n';
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        Ptr<MobilityModel> mob = ueNodes.Get(j)->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector3D speed = mob->GetVelocity();
        std::cout << " Node " << j << " | POS: (x=" << pos.x << ", y=" << pos.y << ") | Speed(" << speed.x << ", " << speed.y << ")" << std::endl;
    }
}

void save_user_postions(NodeContainer nodes) {
    std::ofstream pos_file("positions.txt");
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        pos_file << pos.x << " " << pos.y << " " << pos.z << "\n";
    }
    pos_file.close();
}

// move node "smoothly" towards the given position
void move_drones(Ptr<Node> drone, Vector position, double n_vel) {
    double interval = 0.1;
    double new_n_vel = interval * n_vel;

    // get mobility model for drone
    Ptr<MobilityModel> mob = drone->GetObject<MobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance = CalculateDistance(position, m_position);

    // 1meter of accuracy is acceptable
    if(distance > 1) {
        Vector diff = position - m_position;

        double len = diff.GetLength();
        Vector new_pos = m_position + Vector((diff.x / len) * new_n_vel, 
                                             (diff.y / len) * new_n_vel,
                                             (diff.z / len) * new_n_vel);
        // making sure not to overshoot
        if (CalculateDistance(new_pos, position) > CalculateDistance(position, m_position)) {
            new_pos = position;
            return;
        }

        mob->SetPosition(new_pos);
        Simulator::Schedule(Seconds(interval), &move_drones, drone, position, n_vel);
        return;
    }
    LOG("drone arrived at " << Simulator::Now().GetSeconds());
}

std::vector<std::vector<double>> get_cluster_centers() {
    std::ifstream infile("centers.txt");
    std::vector<std::vector<double>> centers;
    double x, y, z;
    while (infile >> x >> y >> z) {
        std::vector<double> tmp;
        tmp.push_back(x);
        tmp.push_back(y);
        tmp.push_back(z);
        centers.push_back(tmp);
    }
    infile.close();
    return centers;
}

void send_drones_to_cluster_centers(NodeContainer nodes, NodeContainer drones) {
    save_user_postions(nodes);
    exec("python3 clustering.py");
    std::vector<std::vector<double>> centers = get_cluster_centers();

    int counter = 0;
    for(auto&& x: centers) {
        move_drones(drones.Get(counter), Vector(x[0], x[1], x[2]), 20);
        counter++;
    }

    Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers, nodes, drones);
}

// initialize drones position
void set_drones(NodeContainer drones) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,200);
    
    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        Ptr<MobilityModel> mob = drones.Get(i)->GetObject<MobilityModel>();
        mob->SetPosition(Vector(distribution(generator), distribution(generator), distribution(generator)));
    }
}


std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

void ThroughputMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
    flowMon->CheckForLostPackets();
    uint32_t LostPacketsum = 0;
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin(); stats != flowStats.end(); ++stats) {
        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats->first);
        std::cout << "Flow ID			: " << stats->first << " ; " << fiveTuple.sourceAddress << " -----> " << fiveTuple.destinationAddress << std::endl;
        std::cout << "Tx Packets = " << stats->second.txPackets << std::endl;
        std::cout << "Rx Packets = " << stats->second.rxPackets << std::endl;
        std::cout << "Lost Packets = " << (stats->second.txPackets) - (stats->second.rxPackets) << std::endl;
        LostPacketsum = (stats->second.txPackets) - (stats->second.rxPackets);
        std::cout << "Packets Delivery Ratio (PDR) = " << (100 * stats->second.rxPackets) / (stats->second.txPackets) << "%" << std::endl;
        std::cout << "Packets Lost Ratio (PLR) = " << ((LostPacketsum * 100) / stats->second.txPackets) << "%" << std::endl;
        std::cout << "Delay = " << (stats->second.delaySum.GetSeconds()) / (stats->second.txPackets) << " Seconds" << std::endl;
        std::cout << "Total Duration		: " << stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Last Received Packet	: " << stats->second.timeLastRxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps" << std::endl;
        std::cout << "---------------------------------------------------------------------------" << std::endl;
    }
    Simulator::Schedule(Seconds(1), &ThroughputMonitor, fmhelper, flowMon);
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node)
{
    static uint16_t m_port = 2000;
    static int request_id = 0;

    Ptr<Ipv4> ipv4 = sender_node->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
    Ipv4Address ipAddr = iaddr.GetLocal();

    EvalvidServerHelper server(m_port);
    server.SetAttribute("SenderTraceFilename", StringValue("st_highway_cif.st"));
    server.SetAttribute("SenderDumpFilename", StringValue("evalvid_sd_" + std::to_string(request_id)));
    server.SetAttribute("PacketPayload", UintegerValue(512));
    ApplicationContainer apps = server.Install(sender_node);
    apps.Start(Seconds(1));

    EvalvidClientHelper client(ipAddr, m_port);
    client.SetAttribute("ReceiverDumpFilename", StringValue("evalvid_rd_" + std::to_string(request_id)));
    apps = client.Install(receiver_node);
    apps.Start(Seconds(1));

    request_id++;
    m_port++;
}

int main(int argc, char* argv[])
{
    bool useCa = false;
    uint32_t numEnb = 3;
    uint32_t numUes = 30;
    uint32_t seedValue = 10000;
    uint32_t SimTime = 30;
    int eNodeBTxPower = 46;

    uint16_t node_remote = 1; // HOST_REMOTO
    CommandLine cmd;
    std::stringstream cmm;
    std::string GetClusterCoordinates;

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    LogComponentEnable("EvalvidClient", LOG_ALL);

    cmd.AddValue("useCa", "Whether to use carrier aggregation.", useCa);
    cmd.AddValue("numEnb", "the radius of the disc where UEs are placed around an eNB", numEnb);
    cmd.AddValue("numUes", "how many UEs are attached to each eNB", numUes);
    cmd.Parse(argc, argv);

    if (useCa) {
        Config::SetDefault("ns3::LteHelper::UseCa", BooleanValue(useCa));
        Config::SetDefault("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue(2));
        Config::SetDefault("ns3::LteHelper::EnbComponentCarrierManager", StringValue("ns3::RrComponentCarrierManager"));
    }

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(node_remote);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    p2ph.EnablePcapAll("zob");
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces;
    internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4Address remoteHostAddr;
    remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(numEnb);
    ueNodes.Create(numUes);

    internet.Install(ueNodes);
    
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    MobilityHelper enb_mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(remoteHost);
    mobility.Install(pgw);
    enb_mobility.Install(enbNodes);
    BuildingsHelper::Install(enbNodes);

    ns3::RngSeedManager::SetSeed(seedValue); //valor de seed para geração de números aleatórios
    cmd.AddValue("seedValue", "random seed value.", seedValue);
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator", //
        "X", StringValue("100"), // The x coordinate of the center of the random position disc.
        "Y", StringValue("100"), // The y coordinate of the center of the random position disc.
        "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=90]")); // A random variable which represents the radius of a position in a random disc.

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(-200, 200, -200, 200)));
    mobility.Install(ueNodes);
    BuildingsHelper::Install(ueNodes);

    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
        Ptr<Node> ueNode = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    lteHelper->Attach(ueDevs);

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(eNodeBTxPower));
    Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(5)); // Default 5


    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        // request_video(ueNodes.Get(i), remoteHost);
    }

    AnimationInterface animator("lte.xml");
    animator.SetMobilityPollInterval(Seconds(1));
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        animator.UpdateNodeDescription(enbNodes.Get(i), "EnodeB" + i);
        animator.UpdateNodeColor(enbNodes.Get(i), 250, 200, 45);
    }
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        animator.UpdateNodeDescription(ueNodes.Get(j), "UE" + j);
        animator.UpdateNodeColor(ueNodes.Get(j), 20, 10, 145);
    }
    for (uint32_t k = 0; k < remoteHostContainer.GetN(); ++k) {
        animator.UpdateNodeDescription(remoteHostContainer.Get(k), "RemoteHost" + k);
        animator.UpdateNodeColor(remoteHostContainer.Get(k), 110, 150, 45);
    }

    lteHelper->EnableTraces();
    lteHelper->EnablePhyTraces();
    lteHelper->EnableUlPhyTraces();
    lteHelper->EnableMacTraces();

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.Install(ueNodes);

    cmm << "python meumeanshift.py";
    GetClusterCoordinates = exec(cmm.str().c_str());
    if (!GetClusterCoordinates.empty()) {
        std::cout << GetClusterCoordinates;
    }

    Simulator::Stop(Seconds(SimTime));

    for (uint32_t ib = 0; ib <= SimTime; ib++) {
        Simulator::Schedule(Seconds(ib), &print_position, ueNodes);
    }

    Simulator::Schedule(Seconds(1), ThroughputMonitor, &flowHelper, flowMonitor);
    Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers, ueNodes, enbNodes);

    // set initial positions of drones
    set_drones(enbNodes);

    // save user positions to file
    save_user_postions(ueNodes);

    Simulator::Run();
    flowMonitor->SerializeToXmlFile("lte_flow_monitor.xml", true, true);

    std::ifstream inFile;
    std::string x;
    int quantidadeCentroids;
    inFile.open("centroids.txt");
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }
    while (inFile >> x) {
        std::cout << "Quantidade de coordenadas totais: " << x << '\n';
        quantidadeCentroids = std::stoi(x);
    }
    inFile.close();

    int indice = 0;
    float vetorFloat[quantidadeCentroids] = { 0 };

    std::istringstream iss(GetClusterCoordinates);
    std::string line;
    while (std::getline(iss, line)) {
        std::cout << "Teste " << line << std::endl;
        vetorFloat[indice] = std::stof(line);
        indice++;
    }

    std::cout << "This is a message: " << vetorFloat[0] << " years old " << std::endl;
    std::cout << "This is a message: " << vetorFloat[1] << " years old " << std::endl;

    for (std::string::size_type i = 0; i < GetClusterCoordinates.length(); i++) {
        if (GetClusterCoordinates[i] != '\n') // If the current char is not the end,
        {
            std::cout << GetClusterCoordinates[i];
        }
        else if (GetClusterCoordinates[i] == '\n') {
            std::cout << std::endl;
        }
    }

    if (!GetClusterCoordinates.empty()) {
    }

    Simulator::Destroy();
    return 0;
}

