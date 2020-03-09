#include <iostream>
#include <tuple>
#include <vector>
#include "Model/model.hpp"

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
void Car::update_decision() {
  // TODO: Implement
  std::cout << "Updating decision" << std::endl;
}

int main(int argc, char const *argv[]) {

  init_experiment init_vals = {
    // 0, // param
    // 0, // param
  };

  Experiment concertina(init_vals);
  concertina.main_loop();

  return 0;
}
