Solution* HighLevelCBS::findAllPathsByLowLevel(){
//     //TODO we can only do this if all against have NO conflicts/constraints. (Relevant for -ma-CBS only)
    Solution* solution = new Solution();
    Map_Structure map = Map_Structure::get_instance();
    std::vector<Agent*> allAgents;
    auto botList = getControllers();
    for(TestController* bot : botList){
        Agent* agent = new Agent();
        agent->setBot(bot);
        auto plan = bot->findOptimalPath();
        agent->createPath(plan);
        allAgents.push_back(agent);
    };
    solution->agents = allAgents;
    return solution;
}