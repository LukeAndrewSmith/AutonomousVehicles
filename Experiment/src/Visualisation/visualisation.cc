#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "visualisation.hpp"

#include <cstdint>
#include "gif.hpp"

/*----------------------------------------------------------------------------*/
/*                             Single Lane                                    */
/*----------------------------------------------------------------------------*/
void write_space_time_Pbm(const std::vector<std::vector<std::vector<float>>> &data, int road_length, int max_it, std::string filename){
    
    std::ofstream file;
    file.open (filename+".pbm");
    file << "P1" << std::endl; // Simple black/white
    file << road_length << " " << max_it << std::endl; // indicate n cols x n rows
    int i;

    for(auto iteration : data) {
        sort(iteration.begin(), iteration.end()); // Writing to file in the order of increasing x_pos so sort is needed
        i=0;
        for(auto car_pos : iteration) {
            int pos = std::min<int>(road_length-1,round(car_pos.front())); // (position is now an index on a grid => integers) && ( pbm => only x positions => .front()) && (ensure x pos is on the grid)
            while (i<pos) {
                file << 0 << " ";
                i++;
            }
            file << 1 << " ";
            i++;
        }
        while (i<road_length) {
            file << 0 << " ";
            i++;
        }
        file << std::endl;
    }

    file.close();
}

/*----------------------------------------------------------------------------*/
/*                              Multi Lane                                    */
/*----------------------------------------------------------------------------*/
void write_lane_changing_gif(const std::vector<std::vector<std::vector<float>>> &data, int road_length, int max_it, std::string filename, float begin_event, float end_event){
    
    int width = road_length;
    int height = 12; // 1 pixel = 1m
    int min_y_pos = 3; // Delimiters for car position in the lanes
    int max_y_pos = 8;      // (3,8 being the position of the car on the road, 12m width, 1m padding, 5m lanes, car in the middle => 3m and 8m from the top)
    int car_width = 2;
    int car_length = 4;
    int outer_padding = 1; // Whitespace outside of the outer lines
    int full_len = width*height*4; // 4 contiguous values for each pixel, r,g,b,luma (luma is ignored)
    int lane_colour = 128;

    filename = filename + ".gif";
    const char *filname_cstr = filename.c_str();
    int delay = 1;
    GifWriter g;
    GifBegin(&g, filname_cstr, width, height, delay);
    
    for(auto iteration : data) {

        std::vector<uint8_t> fullframe(full_len,255);
        
        // Draw Road
        int full_width = width*4; // 4x as 4 contiguous elements containing colour info for each pixel
        // Outer lines
        auto start = fullframe.begin()+(outer_padding*full_width);
        std::fill( start, start+full_width, lane_colour );
        start = fullframe.begin()+((height-outer_padding)*full_width);
        std::fill( start, start+full_width, lane_colour );
        // Middle dashed line
        int line_len = 4*4;
        int line_gap = 3*4;
        start = fullframe.begin()+((height/2)*full_width); //
        for (int i=0; i<full_width-line_len-line_gap; i=i+line_len+line_gap) {
            std::fill( start+i, start+i+line_len, lane_colour );
        }
        // Begin event
        start = fullframe.begin()+(4*begin_event);
        for (int i=0; i<height; i++) {
            std::fill( start+(i*full_width), start+(i*full_width)+8, lane_colour );
        }
        
        // Begin event
        start = fullframe.begin()+(4*end_event);
        for (int i=0; i<height; i++) {
            std::fill( start+(i*full_width), start+(i*full_width)+8, lane_colour );
        }

        // Draw cars
        for(auto car_pos : iteration) {
            int x_pos = std::min<int>(road_length-1,round(car_pos.front()));    // (position is now an index on a grid => integers) && (ensure x pos is on the grid)
            int y_pos = ((car_pos.back()*(max_y_pos-min_y_pos)) + min_y_pos);   // Scale lanes [0,1]->[min_y_pos,max_y_pos]
            int index = ((x_pos*4)+((full_width*y_pos)));                       // car position translated to an index of the image vector
            for (int i=0;i<0+car_width;i++) {                                   // Add width to car
                std::fill(fullframe.begin()+(index+(i*full_width)),fullframe.begin()+(index+(i*full_width))+(car_length*4)-1,0); // Note: +(car_length*4)-1, -1 as the starting index is included
            }
        }

        GifWriteFrame(&g, fullframe.data(), width, height, delay);
    }
    
    GifEnd(&g);
};
