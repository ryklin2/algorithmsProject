"""
Enhanced A* Algorithm with Hierarchical Partitioning for Self-Driving Car Routing

This simplified proof-of-concept demonstrates an optimized routing algorithm for
self-driving electric cars that finds the most profitable and cost-effective route
from pickup to destination while considering battery constraints, traffic conditions,
demand-based pricing, and vehicle availability.
"""

import heapq
import math
from enum import Enum
from typing import Dict, List, Set, Tuple, Optional, NamedTuple, Union

# -----------------------------------------------------------------------------
# CONSTANTS AND CONFIGURATION
# -----------------------------------------------------------------------------

# Battery management constants
BATTERY_RESOLUTION = 10  # Battery discretization in miles (10-mile increments)
MIN_ARRIVAL_BATTERY = 10  # Minimum battery level to arrive at charging station
RECHARGE_THRESHOLD = 50   # Threshold below which recharging is considered

# Electricity consumption factors
BASE_CONSUMPTION_RATE = 0.2  # Miles of range per mile traveled (base)
TRAFFIC_IMPACT_FACTOR = 0.5  # How much traffic affects consumption

# Cost factors
BASE_PRICE = 2.0              # Base price per mile
ELECTRICITY_COST_PER_UNIT = 0.1  # Cost per mile of range
DEMAND_DISCOUNT_FACTOR = 0.2  # How much demand reduces price (inverse surge)

# -----------------------------------------------------------------------------
# ENUMS AND DATA CLASSES
# -----------------------------------------------------------------------------

class RoadType(Enum):
    """Classification of road types with different efficiency characteristics."""
    LOCAL = 1     # Local streets, least efficient
    ARTERIAL = 2  # Major city streets, medium efficiency
    HIGHWAY = 3   # High-speed highways, most efficient
    
    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented

class PartitionType(Enum):
    """Network partitioning levels for hierarchical routing."""
    LOCAL_LEVEL = 1     # Local roads network
    ARTERIAL_LEVEL = 2  # Arterial roads network
    HIGHWAY_LEVEL = 3   # Highway network

class Coordinates:
    """Geographic coordinates for nodes in the network."""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def distance_to(self, other: 'Coordinates') -> float:
        """Calculate Euclidean distance to another point."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class Node:
    """Represents an intersection or point of interest in the road network."""
    def __init__(self, id: int, coords: Coordinates, road_type: RoadType, 
                 has_charging_station: bool = False):
        self.id = id
        self.coords = coords
        self.road_type = road_type
        self.has_charging_station = has_charging_station
        self.is_near_highway = road_type == RoadType.HIGHWAY or False
    
    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.id == other.id
    
    def __hash__(self):
        return hash(self.id)

class Edge:
    """Represents a road segment connecting two nodes in the network."""
    def __init__(self, source: Node, target: Node, distance: float, 
                 traffic_level: float = 0.0, demand_level: float = 0.0, 
                 vehicle_availability: float = 1.0):
        self.source = source
        self.target = target
        self.distance = distance
        self.traffic_level = traffic_level  # 0.0 to 1.0, higher means more traffic
        self.demand_level = demand_level    # 0.0 to 1.0, higher means more demand
        self.vehicle_availability = vehicle_availability  # 0.0 to 1.0, higher means more available
        
        # Determine edge road type (use lower class if road types differ)
        if source.road_type == target.road_type:
            self.road_type = source.road_type
        else:
            self.road_type = source.road_type if source.road_type.value < target.road_type.value else target.road_type

class Graph:
    """Representation of the road network as a graph of nodes and edges."""
    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.edges: Dict[Tuple[int, int], Edge] = {}
        self.adjacency_list: Dict[int, List[int]] = {}
    
    def add_node(self, node: Node):
        """Add a node to the graph."""
        self.nodes[node.id] = node
        if node.id not in self.adjacency_list:
            self.adjacency_list[node.id] = []
    
    def add_edge(self, edge: Edge):
        """Add a directed edge to the graph."""
        self.edges[(edge.source.id, edge.target.id)] = edge
        
        if edge.source.id not in self.adjacency_list:
            self.adjacency_list[edge.source.id] = []
        self.adjacency_list[edge.source.id].append(edge.target.id)
    
    def get_neighbors(self, node_id: int) -> List[int]:
        """Get all neighboring node IDs for a given node."""
        return self.adjacency_list.get(node_id, [])
    
    def get_edge(self, source_id: int, target_id: int) -> Optional[Edge]:
        """Get the edge connecting source to target if it exists."""
        return self.edges.get((source_id, target_id))

class EnhancedState:
    """Enhanced state representation for A* search, including battery level."""
    def __init__(self, node_id: int, battery_level: float, partition_type: PartitionType):
        self.node_id = node_id
        self.battery_level = battery_level
        self.partition_type = partition_type
    
    def __eq__(self, other):
        if not isinstance(other, EnhancedState):
            return False
        return (self.node_id == other.node_id and
                abs(self.battery_level - other.battery_level) < 5 and  # Some tolerance
                self.partition_type == other.partition_type)
    
    def __hash__(self):
        # Discretize battery level for hashing
        discretized_battery = round(self.battery_level / BATTERY_RESOLUTION) * BATTERY_RESOLUTION
        return hash((self.node_id, discretized_battery, self.partition_type))

class PathCosts(NamedTuple):
    """Contains cost calculations for a road segment."""
    electricity: float  # Electricity consumption for this segment
    segment_cost: float  # Cost for this segment

# -----------------------------------------------------------------------------
# PARTITIONING AND PRE-PROCESSING FUNCTIONS
# -----------------------------------------------------------------------------

def partition_network(graph: Graph) -> Dict[PartitionType, Set[int]]:
    """Partition the network based on road types."""
    partitions = {
        PartitionType.LOCAL_LEVEL: set(),
        PartitionType.ARTERIAL_LEVEL: set(),
        PartitionType.HIGHWAY_LEVEL: set()
    }
    
    for node_id, node in graph.nodes.items():
        if node.road_type == RoadType.HIGHWAY:
            partitions[PartitionType.HIGHWAY_LEVEL].add(node_id)
        elif node.road_type == RoadType.ARTERIAL:
            partitions[PartitionType.ARTERIAL_LEVEL].add(node_id)
        else:
            partitions[PartitionType.LOCAL_LEVEL].add(node_id)
    
    return partitions

def find_containing_partition(node_id: int, partitions: Dict[PartitionType, Set[int]]) -> PartitionType:
    """Find which partition contains the given node."""
    for partition_type, nodes in partitions.items():
        if node_id in nodes:
            return partition_type
    return PartitionType.LOCAL_LEVEL  # Default to local level if not found

def find_boundary_nodes(graph: Graph, partitions: Dict[PartitionType, Set[int]]) -> Set[int]:
    """Identify boundary nodes - nodes that connect different partitions."""
    boundary_nodes = set()
    
    for node_id, node in graph.nodes.items():
        node_partition = find_containing_partition(node_id, partitions)
        
        # Check if this node has neighbors in different partitions
        for neighbor_id in graph.get_neighbors(node_id):
            neighbor_partition = find_containing_partition(neighbor_id, partitions)
            if node_partition != neighbor_partition:
                boundary_nodes.add(node_id)
                break
    
    return boundary_nodes

# -----------------------------------------------------------------------------
# COST CALCULATION FUNCTIONS
# -----------------------------------------------------------------------------

def calculate_electricity_consumption(edge: Edge) -> float:
    """
    Calculate electricity consumption for an edge in miles of range.
    Takes into account distance, traffic, and road type efficiency.
    """
    # Base consumption based on distance
    base_consumption = BASE_CONSUMPTION_RATE * edge.distance
    
    # Traffic factor (more traffic = more electricity)
    traffic_factor = 1.0 + (edge.traffic_level * TRAFFIC_IMPACT_FACTOR)
    
    # Road type efficiency factor
    road_type_factor = get_road_type_efficiency_factor(edge.road_type)
    
    # Calculate total consumption
    consumption = base_consumption * traffic_factor * road_type_factor
    
    # Round up to next BATTERY_RESOLUTION increment for safety
    return math.ceil(consumption / BATTERY_RESOLUTION) * BATTERY_RESOLUTION

def get_road_type_efficiency_factor(road_type: RoadType) -> float:
    """
    Get efficiency factor for different road types.
    Highways are most efficient, local roads least efficient.
    """
    if road_type == RoadType.HIGHWAY:
        return 0.8  # More efficient (20% less consumption)
    elif road_type == RoadType.ARTERIAL:
        return 1.0  # Baseline consumption
    else:  # LOCAL
        return 1.2  # Less efficient (20% more consumption)

def calculate_customer_cost(edge: Edge) -> float:
    """
    Calculate customer cost for an edge in dollars.
    Implements inverse surge pricing: more demand = lower price.
    """
    # Base price per distance
    base_price = BASE_PRICE * edge.distance
    
    # Demand factor (more demand = lower price, inverse surge)
    demand_factor = 1.0 / (1.0 + (edge.demand_level * DEMAND_DISCOUNT_FACTOR))
    
    return base_price * demand_factor

def calculate_segment_costs(edge: Edge) -> PathCosts:
    """
    Calculate costs for a road segment.
    Returns electricity consumption and segment cost with vehicle availability factored in.
    Raises ValueError if there are no available vehicles on this edge.
    """
    # Check if vehicles are available - prevent routing entirely if none available
    if edge.vehicle_availability <= 0.0:
        raise ValueError("No vehicles available on this edge")
    
    # Calculate electricity consumption
    electricity = calculate_electricity_consumption(edge)
    
    # Calculate customer cost
    customer_cost = calculate_customer_cost(edge)
    
    # Calculate profit
    profit = customer_cost - (electricity * ELECTRICITY_COST_PER_UNIT)
    
    # Add penalty for low vehicle availability
    # This still makes low availability edges less desirable
    availability_penalty = (1.0 / edge.vehicle_availability - 1.0) * 10.0
    
    # For A* optimization, segment cost is -profit + penalties
    segment_cost = -profit + availability_penalty
    
    return PathCosts(electricity=electricity, segment_cost=segment_cost)

# -----------------------------------------------------------------------------
# A* HEURISTIC FUNCTIONS
# -----------------------------------------------------------------------------

def heuristic(current_node_id: int, destination_id: int, graph: Graph) -> float:
    """
    A* heuristic function that estimates cost to destination.
    Uses straight-line distance as the base heuristic.
    """
    current_node = graph.nodes[current_node_id]
    destination_node = graph.nodes[destination_id]
    
    # Straight-line distance
    distance = current_node.coords.distance_to(destination_node.coords)
    
    # Estimate minimum cost based on distance
    # Using minimum profit as the cost (negative for A* minimization)
    min_consumption_rate = BASE_CONSUMPTION_RATE * 0.8  # Assume highway efficiency
    min_electricity = min_consumption_rate * distance
    min_cost = BASE_PRICE * distance  # Base customer cost
    min_profit = min_cost - (min_electricity * ELECTRICITY_COST_PER_UNIT)
    
    # For A* optimization (minimizing -profit)
    return -min_profit

# -----------------------------------------------------------------------------
# MAIN ROUTING ALGORITHM
# -----------------------------------------------------------------------------

def find_optimal_route(graph: Graph, pickup_id: int, destination_id: int, 
                      battery_capacity: float, current_battery_level: float) -> Optional[Dict]:
    """
    Find the optimal route from pickup to destination using Enhanced A*.
    Considers battery constraints, traffic, demand, and vehicle availability.
    Prevents cycles by tracking visited nodes in each path.
    
    Args:
        graph: Road network graph
        pickup_id: Starting node ID (can be any valid node)
        destination_id: Destination node ID
        battery_capacity: Maximum battery capacity in miles
        current_battery_level: Current battery level in miles
        
    Returns:
        Dictionary with optimal route and metrics, or None if no route exists
    """
    # Validate input nodes
    if pickup_id not in graph.nodes:
        raise ValueError(f"Pickup node {pickup_id} not found in graph")
    if destination_id not in graph.nodes:
        raise ValueError(f"Destination node {destination_id} not found in graph")
    
    vehicles_available = False
    for neighbor_id in graph.get_neighbors(pickup_id):
        edge = graph.get_edge(pickup_id, neighbor_id)
        if edge and edge.vehicle_availability > 0.0:
            vehicles_available = True
            break
    
    if not vehicles_available:
        print(f"No vehicles available at pickup node {pickup_id}. Cannot serve customer.")
        return None
        
    # Partition the network
    partitions = partition_network(graph)
    boundary_nodes = find_boundary_nodes(graph, partitions)
    
    # Identify partitions for pickup and destination
    pickup_partition = find_containing_partition(pickup_id, partitions)
    
    # Initialize data structures
    open_set = []  # Priority queue (f_score, counter, state)
    counter = 0  # Tiebreaker for equal f_scores
    closed_set = set()  # Set of visited states
    came_from = {}  # For path reconstruction
    g_score = {}  # Cost from start to state
    
    # Track path to each state to prevent cycles
    paths = {}  # Map from state to path (list of node IDs) that led to it
    
    # Create start state
    start_state = EnhancedState(pickup_id, current_battery_level, pickup_partition)
    g_score[start_state] = 0.0
    paths[start_state] = [pickup_id]  # Initial path is just the pickup node
    
    # Calculate initial f_score
    f_score = g_score[start_state] + heuristic(pickup_id, destination_id, graph)
    
    # Add start state to open set
    heapq.heappush(open_set, (f_score, counter, start_state))
    counter += 1
    
    while open_set:
        # Get state with lowest f_score
        _, _, current_state = heapq.heappop(open_set)
        
        # Check if destination reached
        if current_state.node_id == destination_id:
            return reconstruct_path(came_from, current_state, g_score, graph, paths)
        
        # Mark as processed
        closed_set.add(current_state)
        
        # Get current path to this state
        current_path = paths[current_state]
        
        # Process neighbors
        for neighbor_id in graph.get_neighbors(current_state.node_id):
            # Skip if this would create a cycle (node already in current path)
            if neighbor_id in current_path:
                continue
                
            # Get the edge
            edge = graph.get_edge(current_state.node_id, neighbor_id)
            
            # Skip if edge doesn't exist
            if edge is None:
                continue
                
            neighbor_partition = find_containing_partition(neighbor_id, partitions)
            
            # Calculate costs - skip edges with no available vehicles
            try:
                costs = calculate_segment_costs(edge)
            except ValueError:
                # No vehicles available on this edge, skip it
                continue
                
            # Check battery feasibility
            if current_state.battery_level < costs.electricity:
                # Cannot reach this neighbor unless it has a charging station
                if not (graph.nodes[neighbor_id].has_charging_station and 
                        current_state.battery_level >= MIN_ARRIVAL_BATTERY):
                    continue  # Skip this neighbor
            
            # Calculate new battery level
            new_battery_level = current_state.battery_level - costs.electricity
            recharge_cost_adjustment = 0.0
            
            # Handle charging if needed
            if (graph.nodes[neighbor_id].has_charging_station and 
                new_battery_level < RECHARGE_THRESHOLD):
                
                # Calculate recharge cost
                amount_to_recharge = battery_capacity - max(0.0, new_battery_level)
                recharge_cost_adjustment = amount_to_recharge * ELECTRICITY_COST_PER_UNIT
                
                # Update battery level after recharging
                new_battery_level = battery_capacity
            
            # Create neighbor state
            neighbor_state = EnhancedState(neighbor_id, new_battery_level, neighbor_partition)
            
            # Skip if already processed
            if neighbor_state in closed_set:
                continue
            
            # Calculate tentative g_score
            tentative_g_score = g_score[current_state] + costs.segment_cost + recharge_cost_adjustment
            
            # If we found a better path to this state
            if neighbor_state not in g_score or tentative_g_score < g_score[neighbor_state]:
                # Record this path
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_g_score
                
                # Update path to this state
                paths[neighbor_state] = current_path + [neighbor_id]
                
                # Calculate f_score
                f_score = tentative_g_score + heuristic(neighbor_id, destination_id, graph)
                
                # Add to open set
                heapq.heappush(open_set, (f_score, counter, neighbor_state))
                counter += 1
    
    # No path found
    return None

def reconstruct_path(came_from, final_state, g_score, graph, paths) -> Dict:
    """
    Reconstruct the path from start to destination and calculate metrics.
    Uses pre-computed paths to avoid having to walk back through came_from.
    """
    # Get the path directly from the paths dictionary
    path = paths[final_state]
    
    # Calculate metrics
    distance = 0.0
    electricity = 0.0
    customer_cost = 0.0
    
    for i in range(len(path) - 1):
        edge = graph.get_edge(path[i], path[i+1])
        
        # Skip if edge is None (should not happen in proper path)
        if edge is None:
            continue
            
        distance += edge.distance
        electricity += calculate_electricity_consumption(edge)
        customer_cost += calculate_customer_cost(edge)
    
    # Calculate profit
    profit = customer_cost - (electricity * ELECTRICITY_COST_PER_UNIT)
    
    return {
        'path': path,
        'distance': distance,
        'electricity': electricity,
        'customer_cost': customer_cost,
        'profit': profit,
        'cost': g_score[final_state]
    }

def count_available_vehicles(graph: Graph, node_id: Optional[int] = None, radius: Optional[int] = None) -> dict:
    """
    Count available vehicles in the network or around a specific node.
    
    Args:
        graph: Road network graph
        node_id: Optional center node to count from (if None, counts entire network)
        radius: How many hops from center node to include (if None, unlimited)
        
    Returns:
        Dictionary with availability statistics
    """
    if node_id is not None and node_id not in graph.nodes:
        raise ValueError(f"Node {node_id} not found in graph")
    
    vehicle_stats = {
        "total_edges": 0,
        "edges_with_vehicles": 0,
        "avg_availability": 0.0,
        "fully_available_edges": 0,
        "partially_available_edges": 0,
        "no_available_edges": 0
    }
    
    # If node_id is specified, do a breadth-first search to the given radius
    if node_id is not None:
        edges_to_check = set()
        visited = set([node_id])
        queue = [(node_id, 0)]  # (node, distance)
        
        while queue:
            current_node, distance = queue.pop(0)
            
            # If we've reached the radius limit, don't expand further
            if radius is not None and distance >= radius:
                continue
            
            # Add all edges from this node to the set to check
            for neighbor_id in graph.get_neighbors(current_node):
                edge_key = (current_node, neighbor_id)
                edges_to_check.add(edge_key)
                
                # Add neighbor to queue if not visited
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    queue.append((neighbor_id, distance + 1))
    else:
        # Check all edges in the graph
        edges_to_check = [(src, dst) for (src, dst) in graph.edges.keys()]
    
    # Analyze the selected edges
    availability_sum = 0.0
    for src, dst in edges_to_check:
        edge = graph.get_edge(src, dst)
        if edge:
            vehicle_stats["total_edges"] += 1
            availability_sum += edge.vehicle_availability
            
            if edge.vehicle_availability > 0.0:
                vehicle_stats["edges_with_vehicles"] += 1
                
                if edge.vehicle_availability >= 0.99:  # Nearly full availability
                    vehicle_stats["fully_available_edges"] += 1
                else:
                    vehicle_stats["partially_available_edges"] += 1
            else:
                vehicle_stats["no_available_edges"] += 1
    
    # Calculate average availability
    if vehicle_stats["total_edges"] > 0:
        vehicle_stats["avg_availability"] = availability_sum / vehicle_stats["total_edges"]
    
    return vehicle_stats

# -----------------------------------------------------------------------------
# TEST GENERATION AND EXAMPLE USAGE
# -----------------------------------------------------------------------------

def create_test_graph():
    """
    Create a test graph for demonstration.
    This creates a simple road network with varied road types,
    traffic conditions, demand levels, and charging stations.
    """
    graph = Graph()
    
    # Create nodes
    nodes = [
        # id, x, y, road_type, has_charging
        Node(1, Coordinates(0, 0), RoadType.LOCAL, True),  # Starting point
        Node(2, Coordinates(10, 0), RoadType.LOCAL, False),
        Node(3, Coordinates(20, 0), RoadType.ARTERIAL, False),
        Node(4, Coordinates(30, 0), RoadType.ARTERIAL, True),  # Charging station
        Node(5, Coordinates(40, 0), RoadType.HIGHWAY, False),
        Node(6, Coordinates(50, 0), RoadType.HIGHWAY, False),
        Node(7, Coordinates(60, 0), RoadType.HIGHWAY, True),  # Charging station
        Node(8, Coordinates(70, 0), RoadType.ARTERIAL, False),
        Node(9, Coordinates(80, 0), RoadType.LOCAL, False),
        Node(10, Coordinates(90, 0), RoadType.LOCAL, True),  # End point
    ]
    
    # Add nodes to graph
    for node in nodes:
        graph.add_node(node)
    
    # Create bidirectional edges with traffic and demand
    for i in range(len(nodes) - 1):
        # Forward edge
        graph.add_edge(Edge(
            nodes[i], nodes[i+1], 10.0,
            traffic_level=0.2 + (i % 5) * 0.1,  # Varied traffic
            demand_level=0.8 - (i % 5) * 0.1,   # Varied demand (inverse of traffic)
            vehicle_availability=0.9 - (i % 3) * 0.2  # Varied availability
        ))
        
        # Backward edge
        graph.add_edge(Edge(
            nodes[i+1], nodes[i], 10.0,
            traffic_level=0.3 + (i % 4) * 0.1,
            demand_level=0.7 - (i % 4) * 0.1,
            vehicle_availability=0.8 - (i % 3) * 0.2
        ))
    
    # Add a few more edges to create alternative paths
    # Shortcut with high traffic
    graph.add_edge(Edge(nodes[2], nodes[7], 50.0, traffic_level=0.9, demand_level=0.8))
    graph.add_edge(Edge(nodes[7], nodes[2], 50.0, traffic_level=0.9, demand_level=0.8))
    
    # Alternative route
    graph.add_edge(Edge(
        nodes[3], nodes[8], 40.0, 
        traffic_level=0.1,  # Low traffic
        demand_level=0.2,   # Low demand
        vehicle_availability=0.5
    ))
    graph.add_edge(Edge(
        nodes[8], nodes[3], 40.0, 
        traffic_level=0.1,
        demand_level=0.2,
        vehicle_availability=0.5
    ))
    
    return graph

def create_grid_test_graph(size=5):
    """
    Create a grid test graph for more complex routing scenarios.
    """
    graph = Graph()
    
    # Create a grid of nodes
    node_id = 1
    nodes = []
    
    for i in range(size):
        for j in range(size):
            x = float(j * 10)
            y = float(i * 10)
            
            # Determine road type
            if i == 2 or j == 2:  # Main roads
                road_type = RoadType.ARTERIAL
            elif i % 2 == 0 and j % 2 == 0:  # Some highways
                road_type = RoadType.HIGHWAY
            else:  # Local roads
                road_type = RoadType.LOCAL
            
            # Add charging stations strategically
            has_charging = False
            if (i == 0 and j == 0) or (i == size-1 and j == size-1) or (i == 2 and j == 2):
                has_charging = True
            
            node = Node(node_id, Coordinates(x, y), road_type, has_charging)
            nodes.append(node)
            graph.add_node(node)
            node_id += 1
    
    # Connect nodes in grid pattern
    for i in range(size):
        for j in range(size):
            current_idx = i * size + j
            current_node = nodes[current_idx]
            
            # Connect to right neighbor if not at right edge
            if j < size - 1:
                right_idx = i * size + (j + 1)
                right_node = nodes[right_idx]
                
                graph.add_edge(Edge(
                    current_node, right_node, 10.0,
                    traffic_level=0.2 + (current_idx % 5) * 0.1,
                    demand_level=0.6 - (current_idx % 5) * 0.1
                ))
                
                graph.add_edge(Edge(
                    right_node, current_node, 10.0,
                    traffic_level=0.2 + (right_idx % 5) * 0.1,
                    demand_level=0.6 - (right_idx % 5) * 0.1
                ))
            
            # Connect to below neighbor if not at bottom edge
            if i < size - 1:
                below_idx = (i + 1) * size + j
                below_node = nodes[below_idx]
                
                graph.add_edge(Edge(
                    current_node, below_node, 10.0,
                    traffic_level=0.3 + (current_idx % 4) * 0.1,
                    demand_level=0.5 - (current_idx % 4) * 0.1
                ))
                
                graph.add_edge(Edge(
                    below_node, current_node, 10.0,
                    traffic_level=0.3 + (below_idx % 4) * 0.1,
                    demand_level=0.5 - (below_idx % 4) * 0.1
                ))
    
    return graph

def run_basic_example():
    """Run a simple example to demonstrate the routing algorithm."""
    # Create test graph
    graph = create_test_graph()
    
    # Set parameters
    pickup_id = 1        # Start at node 1
    destination_id = 10  # End at node 10
    battery_capacity = 100.0  # 100 miles range
    current_battery_level = 60.0  # Starting with 60% battery
    
    # Find optimal route
    result = find_optimal_route(graph, pickup_id, destination_id, 
                               battery_capacity, current_battery_level)
    
    # Print results
    if result:
        print("\n--- OPTIMAL ROUTE RESULTS ---")
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Customer Cost: ${result['customer_cost']:.2f}")
        print(f"Company Profit: ${result['profit']:.2f}")
        
        # Check if path includes charging
        charging_stations = [node_id for node_id in result['path'] 
                           if graph.nodes[node_id].has_charging_station]
        if len(charging_stations) > 1:  # More than just the starting point
            print("\nRoute includes charging at nodes:", charging_stations)
    else:
        print("No valid route found.")

def run_low_battery_test():
    """Test routing with low battery to demonstrate charging behavior."""
    print("\n=== TEST: ROUTING WITH LOW BATTERY ===")
    graph = create_test_graph()
    result = find_optimal_route(graph, 1, 10, 100.0, 30.0)
    if result:
        print(f"Path with low battery: {result['path']}")
        print(f"Charging stops: {[n for n in result['path'] if graph.nodes[n].has_charging_station]}")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found.")

def run_heavy_traffic_test():
    """Test routing with heavy traffic to demonstrate alternative paths."""
    print("\n=== TEST: ROUTING WITH HEAVY TRAFFIC ===")
    graph = create_test_graph()
    
    # Make the main route very congested
    for i in range(1, 9):
        edge = graph.get_edge(i, i+1)
        if edge:
            edge.traffic_level = 0.9  # Heavy traffic
    
    result = find_optimal_route(graph, 1, 10, 100.0, 80.0)
    if result:
        print(f"Path with heavy traffic: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Uses shortcut or alternative: {'Yes' if set(result['path']) != set(range(1, 11)) else 'No'}")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found.")

def run_zero_availability_test():
    """Test routing with zero vehicle availability at pickup location."""
    print("\n=== TEST: ROUTING WITH ZERO VEHICLE AVAILABILITY ===")
    graph = create_test_graph()
    
    # Test 1: Zero availability on specific edges
    print("\nTest 1: Zero availability on specific edges")
    # Make some edges have zero vehicle availability
    edge1 = graph.get_edge(3, 4)
    edge2 = graph.get_edge(4, 5)
    if edge1:
        edge1.vehicle_availability = 0.0
    if edge2:
        edge2.vehicle_availability = 0.0
    
    # Find route from node 1 to node 10
    result = find_optimal_route(graph, 1, 10, 100.0, 80.0)
    
    if result:
        print(f"Path with zero availability on some edges: {result['path']}")
        uses_edge_3_4 = 3 in result['path'] and 4 in result['path'] and result['path'].index(4) == result['path'].index(3) + 1
        uses_edge_4_5 = 4 in result['path'] and 5 in result['path'] and result['path'].index(5) == result['path'].index(4) + 1
        
        print(f"Uses edge 3->4: {'Yes' if uses_edge_3_4 else 'No'}")
        print(f"Uses edge 4->5: {'Yes' if uses_edge_4_5 else 'No'}")
        print(f"Found alternative route: {'Yes' if result else 'No'}")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found")
    
    # Test 2: Zero availability at pickup location
    print("\nTest 2: Zero availability at pickup location")
    graph = create_test_graph()  # Reset the graph
    
    # Set all edges from pickup to zero availability
    for neighbor_id in graph.get_neighbors(1):
        edge = graph.get_edge(1, neighbor_id)
        if edge:
            edge.vehicle_availability = 0.0
    
    # Get vehicle availability stats
    stats = count_available_vehicles(graph, node_id=1, radius=1)
    print(f"Vehicle availability around pickup node: {stats}")
    
    # Try to find a route (should fail)
    result = find_optimal_route(graph, 1, 10, 100.0, 80.0)
    if result:
        print(f"Path found (unexpected): {result['path']}")
    else:
        print("No vehicles available at pickup - correctly unable to serve customer")

def run_grid_test():
    """Test routing on a grid network to demonstrate hierarchical routing."""
    print("\n=== TEST: GRID NETWORK ROUTING ===")
    graph = create_grid_test_graph(5)  # 5x5 grid
    
    # Start and end at opposite corners
    start_id = 1    # Top-left
    end_id = 25     # Bottom-right
    
    result = find_optimal_route(graph, start_id, end_id, 100.0, 100.0)
    if result:
        print(f"Path across grid: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
        
        # Check road types used
        road_types_used = {}
        for node_id in result['path']:
            road_type = graph.nodes[node_id].road_type
            road_types_used[road_type.name] = road_types_used.get(road_type.name, 0) + 1
        
        print("Road types used in path:", road_types_used)
    else:
        print("No valid route found.")

def run_different_starting_nodes_test():
    """Test routing from different starting nodes in the network."""
    print("\n=== TEST: ROUTING FROM DIFFERENT STARTING NODES ===")
    graph = create_test_graph()
    
    # Test 1: Start from the middle of the network (node 5)
    print("\nTest 1: Start from node 5 (middle of network)")
    result = find_optimal_route(graph, 5, 10, 100.0, 80.0)
    if result:
        print(f"Path from node 5 to 10: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found")
    
    # Test 2: Start from a charging station (node 7)
    print("\nTest 2: Start from node 7 (has charging station)")
    result = find_optimal_route(graph, 7, 1, 100.0, 30.0)  # Low battery to test charging
    if result:
        print(f"Path from node 7 to 1: {result['path']}")
        print(f"Charging stops: {[n for n in result['path'] if graph.nodes[n].has_charging_station]}")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found")
    
    # Test 3: Start near the end and go to the beginning
    print("\nTest 3: Start from node 9 (near end) to node 1 (beginning)")
    result = find_optimal_route(graph, 9, 1, 100.0, 100.0)
    if result:
        print(f"Path from node 9 to 1: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Road types used: {set(graph.nodes[n].road_type.name for n in result['path'])}")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found")

def run_cross_network_test():
    """Test routing across different parts of the grid network."""
    print("\n=== TEST: CROSS-NETWORK ROUTING ===")
    graph = create_grid_test_graph(5)  # 5x5 grid
    
    # Test diagonal routing (top-right to bottom-left)
    start_id = 5    # Top-right corner
    end_id = 21     # Bottom-left corner
    
    print("\nDiagonal routing: top-right to bottom-left")
    result = find_optimal_route(graph, start_id, end_id, 100.0, 100.0)
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
        
        # Check road types used
        road_types_used = {}
        for node_id in result['path']:
            road_type = graph.nodes[node_id].road_type
            road_types_used[road_type.name] = road_types_used.get(road_type.name, 0) + 1
        
        print("Road types used in path:", road_types_used)
    else:
        print("No valid route found")
    
    # Test routing from middle to edge
    print("\nRouting from middle to edge")
    result = find_optimal_route(graph, 13, 5, 100.0, 100.0)  # From center to top-right
    if result:
        print(f"Path from center to top-right: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Profit: ${result['profit']:.2f}")
    else:
        print("No valid route found")

def create_simple_urban_graph(size=5, area_size=0.5):
    """
    Create a small urban test graph with high traffic and many nodes.
    Uses smaller area size and more charging stations.
    """
    graph = Graph()
    
    # Calculate spacing between nodes (smaller for dense urban areas)
    spacing = area_size / (size - 1) if size > 1 else 1.0
    
    # Create nodes in a grid pattern
    nodes = []
    node_id = 1
    
    for i in range(size):
        for j in range(size):
            x = j * spacing
            y = i * spacing
            
            # Determine road type (mostly local roads in urban areas)
            if i % 3 == 0 or j % 3 == 0:  # Major streets
                road_type = RoadType.ARTERIAL
            elif i % 5 == 0 and j % 5 == 0:  # Few highways/expressways
                road_type = RoadType.HIGHWAY
            else:
                road_type = RoadType.LOCAL
            
            # Add more charging stations (every 3 nodes instead of 4)
            has_charging = (i % 3 == 0 and j % 3 == 0)
            
            node = Node(node_id, Coordinates(x, y), road_type, has_charging)
            nodes.append(node)
            graph.add_node(node)
            node_id += 1
    
    # Connect nodes in grid pattern with moderately high traffic
    for i in range(size):
        for j in range(size):
            current_idx = i * size + j
            current_node = nodes[current_idx]
            
            # Connect to right neighbor if not at right edge
            if j < size - 1:
                right_idx = i * size + (j + 1)
                right_node = nodes[right_idx]
                
                # Urban traffic is high but not excessive
                traffic = min(0.8, 0.3 + ((i + j) % 6) * 0.08)
                
                graph.add_edge(Edge(
                    current_node, right_node, spacing,
                    traffic_level=traffic,
                    demand_level=0.3 + ((i + j) % 5) * 0.1,
                    vehicle_availability=0.7 - ((i + j) % 6) * 0.1
                ))
                
                graph.add_edge(Edge(
                    right_node, current_node, spacing,
                    traffic_level=traffic + 0.05,
                    demand_level=0.3 + ((j + i) % 5) * 0.1,
                    vehicle_availability=0.7 - ((j + i) % 6) * 0.1
                ))
            
            # Connect to below neighbor if not at bottom edge
            if i < size - 1:
                below_idx = (i + 1) * size + j
                below_node = nodes[below_idx]
                
                # Urban traffic is high but not excessive
                traffic = min(0.8, 0.3 + ((i + j) % 5) * 0.08)
                
                graph.add_edge(Edge(
                    current_node, below_node, spacing,
                    traffic_level=traffic,
                    demand_level=0.3 + ((i + j) % 4) * 0.1,
                    vehicle_availability=0.7 - ((i + j) % 5) * 0.1
                ))
                
                graph.add_edge(Edge(
                    below_node, current_node, spacing,
                    traffic_level=traffic + 0.05,
                    demand_level=0.3 + ((j + i) % 4) * 0.1,
                    vehicle_availability=0.7 - ((j + i) % 5) * 0.1
                ))
    
    # Add some diagonal connections for more routing options
    for i in range(size - 1):
        for j in range(size - 1):
            current_idx = i * size + j
            diag_idx = (i + 1) * size + (j + 1)
            
            # Only add diagonal if pattern matches (deterministic alternative to random)
            if (i + j) % 3 == 0:
                current_node = nodes[current_idx]
                diag_node = nodes[diag_idx]
                
                diag_distance = spacing * 1.414  # Diagonal distance (√2 * spacing)
                
                # Diagonal connections with moderate traffic
                traffic = min(0.8, 0.4 + ((i + j) % 5) * 0.08)
                
                graph.add_edge(Edge(
                    current_node, diag_node, diag_distance,
                    traffic_level=traffic,
                    demand_level=0.2 + ((i + j) % 4) * 0.1,
                    vehicle_availability=0.6 - ((i + j) % 4) * 0.1
                ))
    
    return graph

def run_urban_test_1():
    """Test routing in a dense urban grid with heavy traffic."""
    print("\n=== URBAN TEST 1: DENSE GRID WITH HEAVY TRAFFIC ===")
    
    # Create a dense 5x5 urban grid in a 0.5-mile square area (25 nodes)
    graph = create_simple_urban_graph(size=5, area_size=0.5)
    
    # Find route from one corner to the opposite corner
    start_id = 1
    end_id = 25
    
    # Use a larger battery capacity for the urban environment
    result = find_optimal_route(graph, start_id, end_id, 30.0, 30.0)
    
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Customer Cost: ${result['customer_cost']:.2f}")
        print(f"Profit: ${result['profit']:.2f}")
        print(f"Profit per mile: ${result['profit']/result['distance']:.2f}")
        
        # Analyze route characteristics
        arterial_count = sum(1 for node_id in result['path'] if graph.nodes[node_id].road_type == RoadType.ARTERIAL)
        highway_count = sum(1 for node_id in result['path'] if graph.nodes[node_id].road_type == RoadType.HIGHWAY)
        local_count = sum(1 for node_id in result['path'] if graph.nodes[node_id].road_type == RoadType.LOCAL)
        
        print(f"Road types: {arterial_count} arterial, {highway_count} highway, {local_count} local")
        print(f"Path length: {len(result['path'])} nodes")
    else:
        print("No valid route found")

def run_urban_test_2():
    """Test routing during rush hour with high traffic on major roads."""
    print("\n=== URBAN TEST 2: RUSH HOUR CONGESTION ===")
    
    graph = create_simple_urban_graph(size=5, area_size=0.5)
    
    # Add rush hour congestion - higher traffic on arterial and highway roads
    for edge_key, edge in graph.edges.items():
        src_node = graph.nodes[edge.source.id]
        dst_node = graph.nodes[edge.target.id]
        
        # Main roads get congestion during rush hour (but not excessive)
        if src_node.road_type == RoadType.ARTERIAL or dst_node.road_type == RoadType.ARTERIAL:
            edge.traffic_level = min(0.9, edge.traffic_level + 0.3)
        elif src_node.road_type == RoadType.HIGHWAY or dst_node.road_type == RoadType.HIGHWAY:
            edge.traffic_level = min(0.9, edge.traffic_level + 0.2)
    
    # Route from middle to edge of city
    start_id = 13  # Middle node
    end_id = 5     # Edge node
    
    result = find_optimal_route(graph, start_id, end_id, 30.0, 30.0)
    
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
        
        # Calculate average traffic on the route
        traffic_levels = []
        for i in range(len(result['path']) - 1):
            edge = graph.get_edge(result['path'][i], result['path'][i+1])
            if edge:
                traffic_levels.append(edge.traffic_level)
        
        avg_traffic = sum(traffic_levels) / len(traffic_levels) if traffic_levels else 0
        print(f"Average traffic on route: {avg_traffic:.2f}")
        print(f"Path length: {len(result['path'])} nodes")
    else:
        print("No valid route found")

def run_urban_test_3():
    """Test routing with some road closures."""
    print("\n=== URBAN TEST 3: ROAD CLOSURES ===")
    
    graph = create_simple_urban_graph(size=5, area_size=0.5)
    
    # Simulate road closures but not too many to create disconnected sections
    closed_roads = 0
    for edge_key, edge in graph.edges.items():
        # Close roads based on a pattern of node IDs (fewer closures)
        source_id = edge.source.id
        target_id = edge.target.id
        if (source_id + target_id) % 11 == 0:  # Fewer closures with modulo 11
            edge.vehicle_availability = 0.0
            closed_roads += 1
    
    print(f"Number of road closures: {closed_roads}")
    
    # Find route across the city
    start_id = 1
    end_id = 25
    
    result = find_optimal_route(graph, start_id, end_id, 30.0, 30.0)
    
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
        print(f"Path length: {len(result['path'])} nodes")
        
        # Analyze if route is longer due to detours
        start_coords = graph.nodes[start_id].coords
        end_coords = graph.nodes[end_id].coords
        spacing = (graph.nodes[2].coords.x - graph.nodes[1].coords.x) if len(graph.nodes) > 1 else 1.0
        
        straight_path_nodes = max(abs(start_coords.x - end_coords.x),
                                 abs(start_coords.y - end_coords.y)) / spacing + 1
        
        detour_factor = len(result['path']) / straight_path_nodes
        print(f"Detour factor: {detour_factor:.2f}x")
    else:
        print("No valid route found - too many road closures")

def run_urban_test_4():
    """Test routing with low battery in a congested downtown area."""
    print("\n=== URBAN TEST 4: LOW BATTERY IN DOWNTOWN ===")
    
    graph = create_simple_urban_graph(size=6, area_size=0.5)  # 36 nodes
    
    # Make traffic moderately high in downtown
    for edge_key, edge in graph.edges.items():
        edge.traffic_level = min(0.85, edge.traffic_level + 0.2)
    
    # Start with low battery but enough to reach charging
    start_id = 8
    end_id = 29
    battery_capacity = 30.0
    current_battery_level = 10.0  # Low battery but not critically low
    
    result = find_optimal_route(graph, start_id, end_id, battery_capacity, current_battery_level)
    
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Profit: ${result['profit']:.2f}")
        print(f"Path length: {len(result['path'])} nodes")
        
        # Identify charging stops
        charging_stops = [node_id for node_id in result['path'] 
                        if graph.nodes[node_id].has_charging_station]
        print(f"Charging stops: {charging_stops}")
        
        # Analyze if route prioritized charging over traffic
        if charging_stops:
            # Calculate average traffic around charging stations vs. overall route
            charging_adjacent_traffic = []
            for i, node_id in enumerate(result['path']):
                if node_id in charging_stops and i > 0 and i < len(result['path'])-1:
                    # Get edges before and after charging station
                    prev_edge = graph.get_edge(result['path'][i-1], node_id)
                    next_edge = graph.get_edge(node_id, result['path'][i+1])
                    if prev_edge:
                        charging_adjacent_traffic.append(prev_edge.traffic_level)
                    if next_edge:
                        charging_adjacent_traffic.append(next_edge.traffic_level)
            
            avg_charging_traffic = sum(charging_adjacent_traffic) / len(charging_adjacent_traffic) if charging_adjacent_traffic else 0
            
            # Get overall route traffic for comparison
            route_traffic = []
            for i in range(len(result['path']) - 1):
                edge = graph.get_edge(result['path'][i], result['path'][i+1])
                if edge:
                    route_traffic.append(edge.traffic_level)
            
            avg_route_traffic = sum(route_traffic) / len(route_traffic) if route_traffic else 0
            
            print(f"Avg traffic near charging stations: {avg_charging_traffic:.2f}")
            print(f"Avg traffic on entire route: {avg_route_traffic:.2f}")
    else:
        print("No valid route found")

def run_urban_test_5():
    """Test routing with inverse surge pricing in high-demand areas."""
    print("\n=== URBAN TEST 5: HIGH DEMAND ZONES ===")
    
    graph = create_simple_urban_graph(size=5, area_size=0.5)
    
    # Create high-demand zones (e.g., downtown business district)
    # Higher demand = lower prices due to inverse surge pricing
    center_x, center_y = 0.25, 0.25  # Center of high-demand zone
    for edge_key, edge in graph.edges.items():
        # Calculate distance from high-demand center
        source_node = graph.nodes[edge.source.id]
        target_node = graph.nodes[edge.target.id]
        
        source_dist = math.sqrt((source_node.coords.x - center_x)**2 + (source_node.coords.y - center_y)**2)
        target_dist = math.sqrt((target_node.coords.x - center_x)**2 + (target_node.coords.y - center_y)**2)
        
        avg_dist = (source_dist + target_dist) / 2
        
        # Higher demand (lower prices) closer to center
        if avg_dist < 0.15:
            edge.demand_level = min(0.99, 0.9)  # Very high demand
        elif avg_dist < 0.3:
            edge.demand_level = min(0.99, 0.7)  # Moderate demand
    
    # Find route that may pass through or avoid high-demand areas
    start_id = 1
    end_id = 25
    
    result = find_optimal_route(graph, start_id, end_id, 30.0, 30.0)
    
    if result:
        print(f"Path: {result['path']}")
        print(f"Distance: {result['distance']:.2f} miles")
        print(f"Electricity Used: {result['electricity']:.2f} miles of range")
        print(f"Customer Cost: ${result['customer_cost']:.2f}")
        print(f"Profit: ${result['profit']:.2f}")
        print(f"Profit per mile: ${result['profit']/result['distance']:.2f}")
        print(f"Path length: {len(result['path'])} nodes")
        
        # Analyze demand levels along the route
        demand_levels = []
        for i in range(len(result['path']) - 1):
            edge = graph.get_edge(result['path'][i], result['path'][i+1])
            if edge:
                demand_levels.append(edge.demand_level)
        
        avg_demand = sum(demand_levels) / len(demand_levels) if demand_levels else 0
        print(f"Average demand on route: {avg_demand:.2f}")
        print(f"Route preference: {'Favors high-demand areas' if avg_demand > 0.5 else 'Avoids high-demand areas'}")
    else:
        print("No valid route found")

def run_urban_tests():
    """Run all urban test cases."""
    print("\n====== URBAN ROUTING TESTS ======")
    
    # Run all urban tests
    run_urban_test_1()
    run_urban_test_2()
    run_urban_test_3()
    run_urban_test_4()
    run_urban_test_5()

def run_all_tests():
    """Run all test cases to demonstrate the algorithm."""
    # Basic example
    print("\n=== BASIC ROUTING EXAMPLE ===")
    run_basic_example()
    
    # Low battery test
    run_low_battery_test()
    
    # Heavy traffic test
    run_heavy_traffic_test()
    
    # Zero vehicle availability test
    run_zero_availability_test()
    
    # Different starting nodes tests
    run_different_starting_nodes_test()
    
    # Cross-network routing test
    run_cross_network_test()
    
    # Grid network test
    run_grid_test()
    
    # Urban routing tests
    run_urban_tests()

# Run all tests
if __name__ == "__main__":
    run_all_tests()