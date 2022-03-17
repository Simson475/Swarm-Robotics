std::vector<TestController*> HighLevelCBS::getControllers(){
    if (botAmount != 0){ return controllers; }
    // Get controllers
    auto &tBotMap = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    controllers = {};
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        TestController* controller = dynamic_cast<TestController*>(&(pcBot->GetControllableEntity().GetController()));
        controllers.push_back(controller);
    }
    
    return controllers;
}