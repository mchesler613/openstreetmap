#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include <stdio.h> // required for fflush
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

//----------------------
// reusable function 
// to process user inputs
//----------------------
void getinput(const std::string name, float &input, int min, int max)
{
    bool keeptrying = false;
    do
    {
        std::cout << name << ":";
        std::cin >> input;
        // std::cout << std::cin.fail() << "\n";
        if (std::cin.fail())
        {
            std::cout << name << " is not a number! Try again.\n";
            keeptrying = true;
            std::cin.clear();
            char buf[BUFSIZ];
            std::cin >> buf;
        }
        else
        {
            keeptrying = false;
            if (input < min || input > max)
                std::cout << name << " is out of bounds.\n";
        }
    }
    while (keeptrying || (input < min || input > max));
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;    
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below.

    float start_x=-1.0, start_y=-1.0, end_x=-1.0, end_y=-1.0;

    // get start_x, start_y, end_x and end_y from user input
    std::cout << "Please enter start_x, start_y, end_x and end_y each between 0 and 100\n";

    getinput("start_x", start_x, 0, 100);
    getinput("start_y", start_y, 0, 100);
    getinput("end_x", end_x, 0, 100);
    getinput("end_y", end_y, 0, 100);

    // Build Model.
    RouteModel model{osm_data};

    // Perform search and render results.
    // RoutePlanner route_planner{model, 10, 10, 90, 90};
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    
    // Call the A* search method on the route_planner
    route_planner.AStarSearch();

    // print the length of path
    std::cout << "Length of Path from ("
        << start_x << "," << start_y << ") to ("
        << end_x << "," << end_y << ") = "
        << route_planner.GetDistance() << std::endl;

    // render the Model
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
