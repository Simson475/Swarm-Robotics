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

SingleThreadBotGreedy::SingleThreadBotGreedy() : RobotInterface{}
{}




REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadUppaalBot_controller")