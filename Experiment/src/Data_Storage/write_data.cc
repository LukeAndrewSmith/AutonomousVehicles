#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <regex>
#include <cmath>
#include "write_data.hpp"


std::vector<std::string> read_directory(std::string const &name) {
    std::vector<std::string> file_names;
    DIR* dirp = opendir(name.c_str());
    struct dirent *dp;
    if (dirp != NULL) {
        while ((dp = readdir(dirp)) != NULL) {
            if ( std::string(dp->d_name).compare(0,10,"experiment") == 0 ) { // Files beginning with experiment (first 10 characters = experiment)
                file_names.push_back(dp->d_name);
            }
        }
        closedir(dirp);
    }
    return file_names;
}

std::string get_file_name(std::string const &directory, int *experiment_num) {
    std::vector<std::string> file_names = read_directory(directory);
    std::vector<int> experiment_nums;
    if (file_names.size() == 0) {
        *experiment_num = 1;
        return "experiment_1";
    } else {
        for (std::string s : file_names) {
            std::string num = std::regex_replace(s, std::regex("[^0-9]+"), ""); // Remove all non-numbers
            experiment_nums.push_back(std::stoi(num));
        }
        int max_val = *max_element(std::begin(experiment_nums), std::end(experiment_nums));
        *experiment_num = max_val+1;
        return "experiment_" + std::to_string(max_val+1);
    }
}


void write_results(std::string filename, float n_cars, float time_taken) {
    std::ofstream data_file;
    std::string file_name = filename + ".txt";
    data_file.open(file_name);

    data_file << "N cars: " << n_cars << std::endl;
    data_file << "Time: " << time_taken << std::endl;
    
    data_file.close();
}
