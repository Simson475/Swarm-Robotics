#include "SingleThreadBotGreedy.hpp"
#include "argos_wrapper/argos_wrapper.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>
#include <ctime>
#include <chrono>



REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadBotGreedy_controller")