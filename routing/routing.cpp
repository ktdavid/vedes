#include <map>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <cstddef>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/index/map/vector.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/geom/haversine.hpp>

#include <boost/config.hpp>
#include <boost/graph/properties.hpp> 
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

typedef double Weight;

typedef boost::property<boost::vertex_name_t, osmium::unsigned_object_id_type> NameProperty;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, NameProperty, WeightProperty> MyGraph;

typedef boost::graph_traits<MyGraph>::vertex_descriptor Vertex;

typedef boost::property_map<MyGraph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_map<MyGraph, boost::vertex_name_t>::type NameMap; 

typedef boost::iterator_property_map<Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
typedef boost::iterator_property_map<Weight*, IndexMap, Weight, Weight&> DistanceMap;

typedef std::map<osmium::unsigned_object_id_type, Vertex> ids_and_vertices;

typedef std::vector<MyGraph::edge_descriptor> RouteType;

osmium::index::map::VectorBasedSparseMap<osmium::unsigned_object_id_type, osmium::Location, std::vector> Locs;
      
class NearestNode : public osmium::handler::Handler
{
    const osmium::Location& location;

    double min_dist = 100000.0;

    osmium::unsigned_object_id_type node_id;

public:
  
    NearestNode( const osmium::Location& loc ) : location( loc ) { }

    void way( const osmium::Way& way )
    {      
        for ( const osmium::NodeRef& nr : way.nodes() ) 
        {
            osmium::Location tmpLocation = Locs.get( nr.positive_ref() );
            double dx = location.lon() - tmpLocation.lon();
            double dy = location.lat() - tmpLocation.lat();
            double distance = dx * dx + dy * dy;

            if ( distance < min_dist )
            {
                min_dist = distance;
                node_id = nr.positive_ref();
            }
        }
    }

    osmium::unsigned_object_id_type get() const
    {
        return node_id;
    }
};

class Route : public osmium::handler::Handler
{
    osmium::memory::Buffer& buffer_first;
    MyGraph g;
    ids_and_vertices where_are_we;
    RouteType path;
  
public:
  
    Route( osmium::memory::Buffer& buffer ) : buffer_first( buffer ) { } 
  
    void way( const osmium::Way& way )
    {
        osmium::NodeRef previous_nr;
        Vertex u, v;
        
        for (const osmium::NodeRef& actual_nr : way.nodes()) 
        {
            ids_and_vertices::iterator vertexIt;
            bool isInserted;
            boost::tie( vertexIt, isInserted ) = where_are_we.emplace( actual_nr.positive_ref(), Vertex() );
                  
            if (!isInserted)
            {
                v = where_are_we[actual_nr.positive_ref()]; 
            }
            else
            {
                v = boost::add_vertex( actual_nr.positive_ref(), g );
                vertexIt->second = v;
            }
              
            if( previous_nr.positive_ref() )
            {
                osmium::geom::Coordinates prev_coords( Locs.get( previous_nr.positive_ref() ) );
                osmium::geom::Coordinates coords( Locs.get( actual_nr.positive_ref() ) );
                Weight w = osmium::geom::haversine::distance( coords, prev_coords );
                boost::add_edge( u, v, w, g );
            }
               
            previous_nr = actual_nr;
            u = v;
        }      
    }
  
    void set_path( const osmium::Location& locStart, const osmium::Location& locDest )
    {  
        NearestNode nodeStart( locStart );
        osmium::apply( buffer_first, nodeStart );
        Vertex start = where_are_we.at( nodeStart.get() );

        NearestNode nodeDest( locDest );
        osmium::apply( buffer_first, nodeDest );
        Vertex dest = where_are_we.at( nodeDest.get() );
        
        std::vector<Vertex> predecessors( boost::num_vertices( g ) );
        std::vector<Weight> distances( boost::num_vertices( g ) );
      
        IndexMap indexMap = boost::get( boost::vertex_index, g );
        PredecessorMap predecessorMap( &predecessors[0], indexMap );
        DistanceMap distanceMap( &distances[0], indexMap );
      
        boost::dijkstra_shortest_paths( g, start, boost::distance_map( distanceMap ).predecessor_map( predecessorMap ) );
      
        for( Vertex u = predecessorMap[dest]; u != dest; dest = u, u = predecessorMap[dest] )
        {
            std::pair<MyGraph::edge_descriptor, bool> edgePair = boost::edge( u, dest, g );
            MyGraph::edge_descriptor edge = edgePair.first;
            path.push_back( edge );
        }  
    }
    
    RouteType get_full_path() const
    {
        return path;
    }
    
    MyGraph get_full_graph() const
    {
        return g;
    }
};

void Usage( char* args[] )
{
    std::cout << "Usage of program: " << args[0] << " input_map_file.osm" << std::endl;
}

int main ( int argc, char* args[] )
{
    if( argc != 2 )
    {
        Usage( args );
        exit( 1 );
    }

    double lat, lon;

    std::cout << "Enter the start coordinate's latitude (x): ";
    std::cin >> lat;
    std::cout << "Enter the start coordinate's longitude (y): ";
    std::cin >> lon;
    // 21.624187, 47.532195
    osmium::Location locStart( lon, lat );

    std::cout << "Enter the destination coordinate's latitude (x): ";
    std::cin >> lat;
    std::cout << "Enter the destination coordinate's longitude (y): ";
    std::cin >> lon;
    // 21.621484, 47.551518
    osmium::Location locDest( lon, lat );

    std::cout << "Calculating route..." << std::endl;
      
    osmium::io::File input_file( args[1] );
    osmium::io::Reader reader( input_file, osmium::osm_entity_bits::all );
    osmium::memory::Buffer buffer = reader.read();

    Route route( buffer );
    
    osmium::handler::NodeLocationsForWays<osmium::index::map::VectorBasedSparseMap<osmium::unsigned_object_id_type, osmium::Location, std::vector>> 
        node_locations( Locs );
    
    osmium::apply( buffer, node_locations );
    osmium::apply( buffer, route );
    
    route.set_path( locStart, locDest );
      
    RouteType path( route.get_full_path() );
    MyGraph g( route.get_full_graph() );
    
    NameMap nameMap = boost::get( boost::vertex_name, g );
    
    RouteType::reverse_iterator routeIterator = path.rbegin();
      
    char yes_or_no;
    std::cout << "Would you like to write the output into a file? \n(if not then terminal will show the result) (y/n) ";
    std::cin >> yes_or_no;

    if(yes_or_no == 'y')
    {
        std::ofstream output_file;

        output_file.open("routing_output.txt");

        if ( routeIterator != path.rend() )
        {
            osmium::Location loc = Locs.get( nameMap[boost::source( *routeIterator, g )] );
            output_file << loc.lon() << ", " << loc.lat() << "\n";
        
            for( ; routeIterator != path.rend(); ++routeIterator )
            {
                loc = Locs.get( nameMap[boost::target( *routeIterator, g )] );
                output_file << loc.lon() << ", " << loc.lat() << "\n";
            }
        }
        else
        {
            output_file << "Unable to find a route between the coordinates you typed in." << std::endl;
        }
        
        output_file.close();     
    }
    else if(yes_or_no == 'n')
    {              
        if ( routeIterator != path.rend() )
        {
            osmium::Location loc = Locs.get( nameMap[boost::source( *routeIterator, g )] );
            std::cout << loc.lon() << ", " << loc.lat() << std::endl;
        
            for( ; routeIterator != path.rend(); ++routeIterator )
            {
                loc = Locs.get( nameMap[boost::target( *routeIterator, g )] );
                std::cout << loc.lon() << ", " << loc.lat() << std::endl;
            }
        }
        else
        {
            std::cout << "Unable to find a route between the coordinates you typed in." << std::endl;
        }
    }

    reader.close();      
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}