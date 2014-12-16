#include <iostream>
#include <cstddef>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/index/map/sparse_table.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>

class BusHandler : public osmium::handler::Handler
{
public:

    osmium::index::map::SparseTable<osmium::unsigned_object_id_type, osmium::Location> Locs;

    int number_of_buses = 0;

    void relation ( osmium::Relation& relation )
    {
        const char* bus = relation.tags()["route"];

        if ( bus && !strcmp ( bus, "bus" ) )
        {
            ++number_of_buses;

            const char* bus_number = relation.tags().get_value_by_key("ref");
            const char* bus_from = relation.tags().get_value_by_key("from");
            const char* bus_to = relation.tags().get_value_by_key("to");
            const char* bus_name = relation.tags().get_value_by_key("name");

            // if(bus_number && !strcmp(bus_number, "22"))
            // {
                // busz adatok kiírása
                std::cout << "\n[" << bus << "]" << std::endl;
                if(bus_number)
                    std::cout << "--------------------\n" << "[Bus Number]: " << "[" << bus_number << "]\n" << "--------------------" << std::endl;
                if(bus_name)
                    std::cout << "Name of the bus: " << bus_name << std::endl;
                if(bus_from || bus_to)
                    std::cout << "[From]: " << bus_from << "\n" << "[To]: " << bus_to << std::endl;
                std::cout << std::endl;
                //

                // megállók koordinátájának és nevének kiírása
                int which = 0;

                osmium::RelationMemberList& relationMemberList = relation.members();

                for ( osmium::RelationMember& relationMember : relationMemberList )
                {
                    if ( !strcmp ( relationMember.role(), "stop" ) && relationMember.type() == osmium::item_type::node )
                    {
                        try
                        {
                            if(which == 0 || which == 20 || which == 30)
                            {
                                osmium::Location loc = Locs.get ( relationMember.ref() );
                                std::cout << ++which << "st STOP = ( " <<  loc.lat() << ", " << loc.lon() << " )" << std::endl;
                            }
                            else if(which == 1 || which == 21 || which == 31)
                            {
                                osmium::Location loc = Locs.get ( relationMember.ref() );
                                std::cout << ++which << "nd STOP = ( " <<  loc.lat() << ", " << loc.lon() << " )" << std::endl;
                            }
                            else if(which == 2 || which == 22 || which == 32)
                            {
                                osmium::Location loc = Locs.get ( relationMember.ref() );
                                std::cout << ++which << "rd STOP = ( " <<  loc.lat() << ", " << loc.lon() << " )" << std::endl;
                            }
                            else
                            {
                                osmium::Location loc = Locs.get ( relationMember.ref() );
                                std::cout << ++which << "th STOP = ( " <<  loc.lat() << ", " << loc.lon() << " )" << std::endl;
                            }
                        }
                        catch ( std::exception& e )
                        {
                            std::cout << "No such node on the map. "<< e.what() << std::endl;
                        }
                    }
                }
            }
        }
    // }
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

      osmium::io::File input_file ( args[1] );
      osmium::io::Reader reader ( input_file, osmium::osm_entity_bits::all );

      BusHandler bus_handler;
      osmium::handler::NodeLocationsForWays<osmium::index::map::SparseTable<osmium::unsigned_object_id_type, osmium::Location>>
           node_locations ( bus_handler.Locs );

      osmium::apply ( reader, node_locations, bus_handler );

      std::cout << "\n------------------------------\nTotal number of buses: " << bus_handler.number_of_buses << "\n------------------------------" << std::endl;

      reader.close();
      google::protobuf::ShutdownProtobufLibrary();
      return 0;
}
