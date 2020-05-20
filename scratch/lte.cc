#include <cstdlib>
#include <iostream>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <random>
#include <list>
#include <limits>
#include <unordered_map>
#include <vector>

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

std::vector<double> ues_sinr;
std::ofstream ues_sinr_file;
std::vector<double> time_to_centroid;
std::ofstream time_to_centroid_file;
unsigned int active_drones = 0;
std::string clustering_algoritm = "kmeans";
bool disableDl = false;
bool disableUl = true;
std::string ns3_dir;

std::string exec(std::string cmd);

void ns3::PhyStatsCalculator::ReportUeSinr(uint16_t cellId, uint64_t imsi, uint16_t rnti, double sinrLinear, uint8_t componentCarrierId)
{
	double sinrdB = 10 * log(sinrLinear);
	ues_sinr[imsi-1] = sinrdB;
}

void CourseChange(std::string context, Ptr<const MobilityModel> model){
	static std::unordered_map<const MobilityModel*, double> start_times;
	double now = Simulator::Now().GetSeconds();
	double start_time;
	if(start_times.count(PeekPointer(model)) > 0){ //check if key exists
		start_time = start_times[PeekPointer(model)];
		if(start_time >= 0){ //check if node started movement
			if(model->GetVelocity().GetLength() == 0){ //Drone stopped?
				time_to_centroid.push_back(now - start_time);
				start_times[PeekPointer(model)] = -1;
				LOG("drone arrived after " << now - start_time << " seconds");
			}
		} else {
			if(model->GetVelocity().GetLength() > 0){ //Drone moving?
				start_times[PeekPointer(model)] = now;
			}
		}
	} else {
		if(model->GetVelocity().GetLength() > 0){ //Drone moving?
			start_times[PeekPointer(model)] = now;
			active_drones++; //New drone moving;
		}
	}
}

void write_metrics(){
	std::stringstream sinrdata;
	unsigned int qtyUEs = ues_sinr.size();
	unsigned int qtyUEsCovered = 0;
	float coverageRatio = 0;
	for(unsigned int id=0; id<qtyUEs; ++id)
	{
		if(ues_sinr[id] >= 3)
			qtyUEsCovered++;
		sinrdata << "Id: "<< id << ", SINR: " << ues_sinr[id] << "dB" << std::endl;
	}
	coverageRatio = (float) qtyUEsCovered/qtyUEs;
	ues_sinr_file << "Coverage ratio: " << coverageRatio * 100 << "%" << std::endl;
	ues_sinr_file << sinrdata.str();

	std::stringstream timedata;
	double total_time = 0;
	double mean_time;
	for(auto time: time_to_centroid){
		total_time += time;
		timedata << time << std::endl;
	}
	mean_time = total_time/active_drones;
	time_to_centroid_file << "Mean time to centroid: " << mean_time << std::endl;
	time_to_centroid_file << timedata.str();
}

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
	// get mobility model for drone
    Ptr<WaypointMobilityModel> mob = drone->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance = CalculateDistance(position, m_position);
	// 1 meter of accuracy is acceptable
	if(distance <= 1)
		return;

	unsigned int nodeId = drone->GetId();
	double currentTime = Simulator::Now().GetSeconds();
	double nWaypointTime;
	LOG("moving drone with nodeId: " << nodeId << " from " << m_position << " to " << position << " time: " << currentTime);

	if(mob->GetVelocity().GetLength() > 0){
		mob->EndMobility();
		mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));
	}

	nWaypointTime = distance/n_vel + currentTime;
	mob->AddWaypoint(Waypoint(Seconds(nWaypointTime), position));

}

std::list<Vector> get_cluster_centers() {
    std::ifstream infile("centers.txt");
    std::list<Vector> centers;
    double x, y, z;
    while (infile >> x >> y >> z) {
        Vector tmp = {x, y, z};
        centers.push_back(tmp);
    }
    infile.close();
    return centers;
}

Vector closest_center(Ptr<Node> drone, std::list<Vector> centers){
	double min = numeric_limits<double>::infinity();
	double distance;
	Vector drone_position = drone->GetObject<MobilityModel>()->GetPosition();
	Vector closest;
	for(auto&& center_position: centers) {
		distance = CalculateDistance(drone_position, center_position);
		if(distance < min){
			min = distance;
			closest = center_position;
		}
	}
	return closest;
}

std::list<Vector>::iterator find_center(Vector center, std::list<Vector>& centers){
	std::list<Vector>::iterator position;
	for(position = centers.begin(); position != centers.end(); ++position) {
		if(position->x == center.x &&
			position->y == center.y &&
			position->z == center.z){
			break;
		}
	}
	return position;
}

void send_drones_to_cluster_centers(NodeContainer nodes, NodeContainer drones) {
    // save user positions to file
    save_user_postions(nodes);
    // generate custering file
    exec(std::string("python3 ") + ns3_dir + std::string("/clustering.py ") + clustering_algoritm);
    // read cluster centers
    std::list<Vector> centers = get_cluster_centers();
	Vector center;
	std::list<Vector>::iterator erase_position;

    // iterate custer centers and send drones
    NodeContainer::Iterator drone;
    for(drone = drones.Begin(); drone != drones.End() && centers.size() > 0; ++drone) {
		center = closest_center(*drone, centers);
		erase_position = find_center(center, centers);
		centers.erase(erase_position);
        move_drones(*drone, center, 20);
    }

    // repeat
    Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers, nodes, drones);
}

// initialize drones position
void set_drones(NodeContainer drones) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,1000);

    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        Ptr<WaypointMobilityModel> mob = drones.Get(i)->GetObject<WaypointMobilityModel>();
		mob->AddWaypoint(Waypoint(Simulator::Now(), Vector(distribution(generator), distribution(generator), 35)));
    }
}


std::string exec(std::string cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
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
	float PDR, PLR, Delay, Throughput;
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
	std::ofstream qos_file;
	qos_file.open("qos.txt", std::ofstream::out | std::ofstream::trunc);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin(); stats != flowStats.end(); ++stats) {
        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats->first);
		PDR = (100 * stats->second.rxPackets) / (stats->second.txPackets);
        LostPacketsum = (stats->second.txPackets) - (stats->second.rxPackets);
		PLR = ((LostPacketsum * 100) / stats->second.txPackets);
		Delay = (stats->second.delaySum.GetSeconds()) / (stats->second.txPackets);
		Throughput = stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;

        std::cout << "Flow ID			: " << stats->first << " ; " << fiveTuple.sourceAddress << " -----> " << fiveTuple.destinationAddress << std::endl;
        std::cout << "Tx Packets = " << stats->second.txPackets << std::endl;
        std::cout << "Rx Packets = " << stats->second.rxPackets << std::endl;
        std::cout << "Lost Packets = " << (stats->second.txPackets) - (stats->second.rxPackets) << std::endl;
        std::cout << "Packets Delivery Ratio (PDR) = " << PDR << "%" << std::endl;
        std::cout << "Packets Lost Ratio (PLR) = " << PLR << "%" << std::endl;
        std::cout << "Delay = " << Delay << " Seconds" << std::endl;
        std::cout << "Total Duration		: " << stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Last Received Packet	: " << stats->second.timeLastRxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Throughput: " << Throughput << " Mbps" << std::endl;
        std::cout << "---------------------------------------------------------------------------" << std::endl;
		qos_file << fiveTuple.sourceAddress << " --> " << fiveTuple.destinationAddress << "," << PDR << "," << PLR << "," << Delay << "," << Throughput << "\n";
    }

	qos_file.close();
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node)
{
    static uint16_t m_port = 2000;
    static int request_id = 0;

    Ptr<Ipv4> ipv4 = sender_node->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
    Ipv4Address ipAddr = iaddr.GetLocal();

    EvalvidServerHelper server(m_port);
    server.SetAttribute("SenderTraceFilename", StringValue(ns3_dir + std::string("/st_highway_cif.st")));
    //server.SetAttribute("SenderDumpFilename", StringValue("evalvid_sd_" + std::to_string(request_id)));
		server.SetAttribute("SenderDumpFilename", StringValue("sd_" + std::to_string(request_id)));
    server.SetAttribute("PacketPayload", UintegerValue(512));
    ApplicationContainer apps = server.Install(sender_node);
    apps.Start(Seconds(5));

    EvalvidClientHelper client(ipAddr, m_port);
    //client.SetAttribute("ReceiverDumpFilename", StringValue("evalvid_rd_" + std::to_string(request_id)));
		client.SetAttribute("ReceiverDumpFilename", StringValue("rd_" + std::to_string(request_id)));
    apps = client.Install(receiver_node);
    apps.Start(Seconds(5));

    request_id++;
    m_port++;
}

void UDPApp (Ptr<Node> remoteHost, NodeContainer ueNodes)
{
	// Install and start applications on UEs and remote host

	ApplicationContainer serverApps;
	ApplicationContainer clientApps;
	Time interPacketInterval = MilliSeconds (50);
	uint16_t dlPort = 1100;
	uint16_t ulPort = 2000;
	int startTime = 2;
	Ptr<Ipv4> remoteIpv4 = remoteHost->GetObject<Ipv4>();
	Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal(); //Interface 0 is loopback

	for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
	{
		Ptr<Node> ue = ueNodes.Get (u);
		Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4>();
		Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();
		ulPort++;

		if (!disableDl)
		{
			PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
			serverApps.Add (dlPacketSinkHelper.Install (ue));

			UdpClientHelper dlClient (ueIpAddr, dlPort);
			dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
			dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
			dlClient.SetAttribute ("PacketSize", UintegerValue (1024));
			clientApps.Add (dlClient.Install (remoteHost));
		}

		if (!disableUl)
		{
			++ulPort;
			PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
			serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

			UdpClientHelper ulClient (remoteIpAddr, ulPort);
			ulClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
			ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
			ulClient.SetAttribute ("PacketSize", UintegerValue (1024));
			clientApps.Add (ulClient.Install (ue));
		}
	}

	serverApps.Start (Seconds(1));
	clientApps.Start (Seconds(startTime));
}

bool IsTopLevelSourceDir (std::string path)
{
	bool haveVersion = false;
	bool haveLicense = false;

	//
	// If there's a file named VERSION and a file named LICENSE in this
	// directory, we assume it's our top level source directory.
	//

	std::list<std::string> files = SystemPath::ReadFiles (path);
	for (std::list<std::string>::const_iterator i = files.begin (); i != files.end (); ++i)
	{
		if (*i == "VERSION")
		{
			haveVersion = true;
		}
		else if (*i == "LICENSE")
		{
			haveLicense = true;
		}
	}

	return haveVersion && haveLicense;
}

std::string GetTopLevelSourceDir (void)
{
	std::string self = SystemPath::FindSelfDirectory ();
	std::list<std::string> elements = SystemPath::Split (self);
	while (!elements.empty ())
	{
		std::string path = SystemPath::Join (elements.begin (), elements.end ());
		if (IsTopLevelSourceDir (path))
		{
			return path;
		}
		elements.pop_back ();
	}
	NS_FATAL_ERROR ("Could not find source directory from self=" << self);
}

int main(int argc, char* argv[])
{
    bool useCa = false;
    uint32_t numUAVs = 3;
    uint32_t numUes = 75;
	uint32_t numCars = 10;
    uint32_t seedValue = 10000;
    uint32_t SimTime = 85;
    int eNodeBTxPower = 23;

    uint16_t node_remote = 1; // HOST_REMOTO
    CommandLine cmd;
    std::stringstream cmm;
    std::string GetClusterCoordinates;
	//Open file for writing and overwrite if it already exists
	ues_sinr_file.open("ues_sinr.txt", std::ofstream::out | std::ofstream::trunc);
	time_to_centroid_file.open("time_to_centroid.txt", std::ofstream::out | std::ofstream::trunc);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    LogComponentEnable("EvalvidClient", LOG_INFO);
	LogComponentEnable("EvalvidServer", LOG_INFO);

    cmd.AddValue("useCa", "Whether to use carrier aggregation.", useCa);
    cmd.AddValue("numUAVs", "how many drones in the simulation", numUAVs);
    cmd.AddValue("numUes", "how many UEs are attached to each eNB", numUes);
    cmd.AddValue("seedValue", "random seed value.", seedValue);
	cmd.AddValue("algo", "clustering algoritm to use", clustering_algoritm);
    cmd.Parse(argc, argv);

    ns3::RngSeedManager::SetSeed(seedValue); //valor de seed para geração de números aleatórios
	ns3_dir = GetTopLevelSourceDir();
    
	if (useCa) {
        Config::SetDefault("ns3::LteHelper::UseCa", BooleanValue(useCa));
        Config::SetDefault("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue(2));
        Config::SetDefault("ns3::LteHelper::EnbComponentCarrierManager", StringValue("ns3::RrComponentCarrierManager"));
    }

	ues_sinr.resize(numUes+numCars);

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

	NS_LOG_UNCOND("Pathloss model: HybridBuildingsPropagationLossModel ");
	Config::SetDefault("ns3::ItuR1411NlosOverRooftopPropagationLossModel::StreetsOrientation", DoubleValue (10));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2.0e9));
	lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0));
	lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (3.0));
	lteHelper->SetPathlossModelAttribute ("Los2NlosThr", DoubleValue (200));
    
	NodeContainer remoteHostContainer;
    remoteHostContainer.Create(node_remote);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    //p2ph.EnablePcapAll("zob");
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
	remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.192"), 1); //Route to UAVs

	Ptr<Ipv4StaticRouting> pgwStaticRouting = ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
	pgwStaticRouting->AddNetworkRouteTo(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.192"), 2);

    NodeContainer UAVNodes;
    NodeContainer ueNodes;
	NodeContainer carNodes;

    UAVNodes.Create(numUAVs);
    ueNodes.Create(numUes);
	carNodes.Create(numCars);

    internet.Install(ueNodes);
	internet.Install(carNodes);
    
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(remoteHost);
    mobility.Install(pgw);
	BuildingsHelper::Install (remoteHost);
	BuildingsHelper::Install (pgw);
    
	MobilityHelper UAVmobility;
    UAVmobility.SetMobilityModel("ns3::WaypointMobilityModel");
    UAVmobility.Install(UAVNodes);
    BuildingsHelper::Install(UAVNodes);

    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
        "X", DoubleValue(500), // The x coordinate of the center of the random position disc.
        "Y", DoubleValue(500), // The y coordinate of the center of the random position disc.
		"Z", DoubleValue(1.5), // The z coordinate of all positions in the disc.
        "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=400]")); // A random variable which represents the radius of a position in a random disc.

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(-1000, 1000, -1000, 1000)));
    mobility.Install(ueNodes);
    BuildingsHelper::Install(ueNodes);

	Ptr<ConstantRandomVariable> Z = CreateObject<ConstantRandomVariable> ();
	Z->SetAttribute ("Constant", DoubleValue (2));

	Ptr<RandomBoxPositionAllocator> waypointAllocator = CreateObject<RandomBoxPositionAllocator> ();
	waypointAllocator->SetAttribute ("X", StringValue("ns3::UniformRandomVariable[Min=-1000|Max=1000]"));
	waypointAllocator->SetAttribute ("Y", StringValue("ns3::UniformRandomVariable[Min=-1000|Max=1000]"));
	waypointAllocator->SetAttribute ("Z", PointerValue (Z));

	mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
		"Speed", StringValue("ns3::UniformRandomVariable[Min=5|Max=12]"),
		"PositionAllocator", PointerValue (waypointAllocator));
	mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
        "X", DoubleValue(500), // The x coordinate of the center of the random position disc.
        "Y", DoubleValue(500), // The y coordinate of the center of the random position disc.
		"Z", DoubleValue(2), // The z coordinate of all positions in the disc.
        "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=400]"));
	mobility.Install(carNodes);
	BuildingsHelper::Install(carNodes);

    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;
	NetDeviceContainer carDevs;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
        Ptr<Node> ueNode = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

	for (uint32_t u = 0; u < carNodes.GetN(); ++u) {
		Ptr<Node> carNode = carNodes.Get(u);
		Ptr<Ipv4StaticRouting> carStaticRouting = ipv4RoutingHelper.GetStaticRouting(carNode->GetObject<Ipv4>());
		carStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
	}

	lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (25)); //Set Download BandWidth
	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (25)); //Set Upload Bandwidth
    enbDevs = lteHelper->InstallEnbDevice(UAVNodes);
    
	ueDevs = lteHelper->InstallUeDevice(ueNodes);
	carDevs = lteHelper->InstallUeDevice(carNodes);

    Ipv4InterfaceContainer ueIpIface;
	Ipv4InterfaceContainer carIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
	carIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(carDevs));

    lteHelper->Attach(ueDevs);
	lteHelper->Attach(carDevs);

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(eNodeBTxPower));
    Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(5)); // Default 5

	for (uint32_t u = 0; u < UAVNodes.GetN(); ++u) {
        Ptr<Node> UAVNode = UAVNodes.Get(u);
		Ptr<Ipv4> UAVIpv4 = UAVNode->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> UAVStaticRouting = ipv4RoutingHelper.GetStaticRouting(UAVIpv4);
        UAVStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    }
	Ptr<Node> sgw = epcHelper->GetSgwNode();
	Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();
	Ptr<Ipv4StaticRouting> sgwStaticRouting = ipv4RoutingHelper.GetStaticRouting(sgwIpv4);
	sgwStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

	//Setup Applications
	UDPApp(remoteHost, NodeContainer(ueNodes, carNodes));

    for (uint32_t i = 0; i < UAVNodes.GetN(); ++i) {
        request_video(UAVNodes.Get(i), remoteHost);
    }

    AnimationInterface animator("lte.xml");
    animator.SetMobilityPollInterval(Seconds(1));
    for (uint32_t i = 0; i < UAVNodes.GetN(); ++i) {
        animator.UpdateNodeDescription(UAVNodes.Get(i), "UAV " + std::to_string(i));
        animator.UpdateNodeColor(UAVNodes.Get(i), 250, 200, 45);
		animator.UpdateNodeSize(UAVNodes.Get(i)->GetId(),10,10); // to change the node size in the animation.
    }
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        animator.UpdateNodeDescription(ueNodes.Get(j), "UE " + std::to_string(j));
        animator.UpdateNodeColor(ueNodes.Get(j), 20, 10, 145);
		animator.UpdateNodeSize(ueNodes.Get(j)->GetId(),10,10);
    }
	for (uint32_t j = 0; j < carNodes.GetN(); ++j) {
		animator.UpdateNodeDescription(carNodes.Get(j), "Car " + std::to_string(j));
		animator.UpdateNodeColor(carNodes.Get(j), 20, 100, 145);
		animator.UpdateNodeSize(carNodes.Get(j)->GetId(),10,10);
	}
    for (uint32_t k = 0; k < remoteHostContainer.GetN(); ++k) {
        animator.UpdateNodeDescription(remoteHostContainer.Get(k), "RemoteHost " + std::to_string(k));
        animator.UpdateNodeColor(remoteHostContainer.Get(k), 110, 150, 45);
		animator.UpdateNodeSize(remoteHostContainer.Get(k)->GetId(),10,10);
    }

    //lteHelper->EnableTraces(); //enable all traces
    //lteHelper->EnablePhyTraces();
    lteHelper->EnableUlPhyTraces();
    //lteHelper->EnableMacTraces();

	Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportUeSinr",MakeCallback (&ns3::PhyStatsCalculator::ReportUeSinr));

	unsigned int id;
	std::ostringstream oss;
	for (uint32_t u = 0; u < UAVNodes.GetN(); ++u) {
		id = UAVNodes.Get(u)->GetId();
		oss << "/NodeList/" << id << "/$ns3::MobilityModel/CourseChange";
		Config::Connect(oss.str(), MakeCallback(&CourseChange));
		oss.str("");
	}

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
	flowHelper.Install(remoteHost);
	flowHelper.Install(UAVNodes);
    flowHelper.Install(ueNodes);
	flowMonitor = flowHelper.Install(carNodes);

    Simulator::Stop(Seconds(SimTime));

    // for (uint32_t ib = 0; ib <= SimTime; ib++) {
    //     Simulator::Schedule(Seconds(ib), &print_position, ueNodes);
    // }

	Simulator::Schedule(Seconds(SimTime-0.001), &write_metrics);
    Simulator::Schedule(Seconds(SimTime-0.001), ThroughputMonitor, &flowHelper, flowMonitor);
    Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers, NodeContainer(ueNodes, carNodes), UAVNodes);

    // set initial positions of drones
    set_drones(UAVNodes);

    // save user positions to file
    save_user_postions(NodeContainer(ueNodes, carNodes));

	BuildingsHelper::MakeMobilityModelConsistent ();

    Simulator::Run();
    flowMonitor->SerializeToXmlFile("lte_flow_monitor.xml", true, true);

    // std::ifstream inFile;
    // std::string x;
    // int quantidadeCentroids;
    // inFile.open("centroids.txt");
    // if (!inFile) {
    //     std::cout << "Unable to open file";
    //     exit(1); // terminate with error
    // }
    // while (inFile >> x) {
    //     std::cout << "Quantidade de coordenadas totais: " << x << '\n';
    //     quantidadeCentroids = std::stoi(x);
    // }
    // inFile.close();

    // int indice = 0;
    // float vetorFloat[quantidadeCentroids] = { 0 };

    // std::istringstream iss(GetClusterCoordinates);
    // std::string line;
    // while (std::getline(iss, line)) {
    //     std::cout << "Teste " << line << std::endl;
    //     vetorFloat[indice] = std::stof(line);
    //     indice++;
    // }

    // std::cout << "This is a message: " << vetorFloat[0] << " years old " << std::endl;
    // std::cout << "This is a message: " << vetorFloat[1] << " years old " << std::endl;

    // for (std::string::size_type i = 0; i < GetClusterCoordinates.length(); i++) {
    //     if (GetClusterCoordinates[i] != '\n') // If the current char is not the end,
    //     {
    //         std::cout << GetClusterCoordinates[i];
    //     }
    //     else if (GetClusterCoordinates[i] == '\n') {
    //         std::cout << std::endl;
    //     }
    // }

    // if (!GetClusterCoordinates.empty()) {
    // }

    Simulator::Destroy();
    return 0;
}

