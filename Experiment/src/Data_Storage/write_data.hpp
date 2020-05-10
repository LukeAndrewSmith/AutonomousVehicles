#include <string>

std::string get_file_name(std::string const &directory, int *experiment_num);
void write_results(std::string filename, float n_cars, float time_taken);
