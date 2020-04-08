#include <string>

void write_data(std::vector<std::vector<std::vector<float>>> const &data, int max_it, int road_length, std::string filename);
std::string get_file_name(std::string const &directory, int &experiment_num);
