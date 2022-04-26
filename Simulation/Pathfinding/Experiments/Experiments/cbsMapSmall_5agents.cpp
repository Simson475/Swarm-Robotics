#include "HighLevelCBS.hpp"

std::shared_ptr<Graph> getCbsMapSmallGraph();

int main(int argc, char *argv[]) {
    Logger& logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "cbsMapSmall_5agents_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int agentCount = 5; agentCount <= 5; ++agentCount){
        std::cout << "Running experiment with " << agentCount << " agents..";
        std::cout.flush();
        std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_cbsMapSmall_5agents.txt";
        // Remove any existing results if they exist
        remove(&experimentResultFile[0]);
        logger.setLogFile(experimentResultFile);

        // Arrange experiment
        auto graph = getCbsMapSmallGraph();
        auto vertices = graph->getVertices();
        int spawnPointVertexIndexOffset = 37;
        std::vector<std::shared_ptr<Vertex>> stations = {
            vertices[6],
            vertices[8],
            vertices[10],
            vertices[12],
            vertices[14],
            vertices[16],
            vertices[18]
        };
        std::vector<std::vector<std::shared_ptr<Vertex>>> agentJobs = {
            {stations[0]},
            {stations[1]},
            {stations[2]},
            {stations[3]},
            {stations[4]}
        };
        std::vector<bool> agentHasFinished = { 0, 0, 0, 0, 0 };
        std::vector<AgentInfo> agents{(long unsigned int)agentCount};
        for (int i = 0; i < agentCount; ++i){
            auto startVertex = vertices[spawnPointVertexIndexOffset + i];
            auto goalVertex = agentJobs[i][0];// TODO set to actual goal
            agents[i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
        }

        // Act
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance(), 0);
        while (true){
            // Find the path that finish first (lowest cost path)
            int agentThatFinishFirst = -1;
            float minCost = std::numeric_limits<float>::infinity();
            for (int i = 0; i < agentCount; ++i){
                if ( ! agentHasFinished[i]){
                    if (solution.paths[i].cost < minCost){
                        minCost = solution.paths[i].cost;
                        agentThatFinishFirst = i;
                    }
                }
            }
            if (agentThatFinishFirst == -1){
                break;// We are done
            }
            // Update job for the agent that finished
            if (agentJobs[agentThatFinishFirst].size() > 0){
                agentJobs[agentThatFinishFirst].erase(agentJobs[agentThatFinishFirst].begin());
            }
            else{
                agentHasFinished[agentThatFinishFirst] = true;
            }
            // Update the agents to have a new current action at the current time
            for (int i = 0; i < agentCount; ++i){
                auto currentAction = solution.paths[i].actions.back();// Action(minCost, solution.paths[i].actions.back().endVertex, solution.paths[i].actions.back().endVertex, 0);
                // Since the above is only true for the finishing agent, we need to find the current action for the rest
                for (auto& action : solution.paths[i].actions){
                    // Get the first action that did not end yet
                    if (action.timestamp + action.duration >= minCost){
                        currentAction = action;
                        break;
                    }
                }
                auto goalVertex = agentJobs[i].size() > 0 ? agentJobs[i].front() : vertices[spawnPointVertexIndexOffset + i];
                bool isWorking = currentAction.isWaitAction() && currentAction.endVertex == goalVertex && currentAction.duration == TIME_AT_GOAL;
                bool shouldWorkAtGoal = (goalVertex != vertices[spawnPointVertexIndexOffset + i]);
                agents[i] = AgentInfo(i, currentAction, goalVertex, isWorking, shouldWorkAtGoal);
            }
            // Find new solution
            if ( ! agentHasFinished[agentThatFinishFirst]){
                solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance(), minCost);
            }
            // Repeat until all jobs are done...
        }
        auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

        // Results
        (*logger.begin()) << "Experiment took "<< experimentTime << "[µs]\n"; logger.end();
        float sumOfCosts = 0;
        int i = 0;
        for (auto path : solution.paths){
            sumOfCosts += path.cost;
            (*logger.begin()) << "Agent" << i++ << " finished its job with cost " << path.cost << "\n"; logger.end();
        }
        (*logger.begin()) << "Sum of costs = "<< sumOfCosts << "\n"; logger.end();
        std::cout << "Done\n";
    }
}

std::shared_ptr<Graph> getCbsMapSmallGraph()
{
    auto v0 = std::make_shared<Vertex>(0);
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    auto v4 = std::make_shared<Vertex>(4);
    auto v5 = std::make_shared<Vertex>(5);
    auto v6 = std::make_shared<Vertex>(6);
    auto v7 = std::make_shared<Vertex>(7);
    auto v8 = std::make_shared<Vertex>(8);
    auto v9 = std::make_shared<Vertex>(9);
    auto v10 = std::make_shared<Vertex>(10);
    auto v11 = std::make_shared<Vertex>(11);
    auto v12 = std::make_shared<Vertex>(12);
    auto v13 = std::make_shared<Vertex>(13);
    auto v14 = std::make_shared<Vertex>(14);
    auto v15 = std::make_shared<Vertex>(15);
    auto v16 = std::make_shared<Vertex>(16);
    auto v17 = std::make_shared<Vertex>(17);
    auto v18 = std::make_shared<Vertex>(18);
    auto v19 = std::make_shared<Vertex>(19);
    auto v20 = std::make_shared<Vertex>(20);
    auto v21 = std::make_shared<Vertex>(21);
    auto v22 = std::make_shared<Vertex>(22);
    auto v23 = std::make_shared<Vertex>(23);
    auto v24 = std::make_shared<Vertex>(24);
    auto v25 = std::make_shared<Vertex>(25);
    auto v26 = std::make_shared<Vertex>(26);
    auto v27 = std::make_shared<Vertex>(27);
    auto v28 = std::make_shared<Vertex>(28);
    auto v29 = std::make_shared<Vertex>(29);
    auto v30 = std::make_shared<Vertex>(30);
    auto v31 = std::make_shared<Vertex>(31);
    auto v32 = std::make_shared<Vertex>(32);
    auto v33 = std::make_shared<Vertex>(33);
    auto v34 = std::make_shared<Vertex>(34);
    auto v35 = std::make_shared<Vertex>(35);
    auto v36 = std::make_shared<Vertex>(36);
    auto v37 = std::make_shared<Vertex>(37);
    auto v38 = std::make_shared<Vertex>(38);
    auto v39 = std::make_shared<Vertex>(39);
    auto v40 = std::make_shared<Vertex>(40);
    auto v41 = std::make_shared<Vertex>(41);
    auto v42 = std::make_shared<Vertex>(42);
    auto v43 = std::make_shared<Vertex>(43);
    auto v44 = std::make_shared<Vertex>(44);
    auto v45 = std::make_shared<Vertex>(45);
    auto v46 = std::make_shared<Vertex>(46);
    auto v47 = std::make_shared<Vertex>(47);
    auto v48 = std::make_shared<Vertex>(48);
    auto v49 = std::make_shared<Vertex>(49);
    auto v50 = std::make_shared<Vertex>(50);
    auto v51 = std::make_shared<Vertex>(51);
    auto v52 = std::make_shared<Vertex>(52);
    auto v53 = std::make_shared<Vertex>(53);
    auto v54 = std::make_shared<Vertex>(54);
    auto v55 = std::make_shared<Vertex>(55);
    auto v56 = std::make_shared<Vertex>(56);
    auto v57 = std::make_shared<Vertex>(57);
    auto v58 = std::make_shared<Vertex>(58);
    auto v59 = std::make_shared<Vertex>(59);
    auto v60 = std::make_shared<Vertex>(60);
    auto v61 = std::make_shared<Vertex>(61);
    auto v62 = std::make_shared<Vertex>(62);
    auto v63 = std::make_shared<Vertex>(63);
    auto v64 = std::make_shared<Vertex>(64);
    auto v65 = std::make_shared<Vertex>(65);
    auto v66 = std::make_shared<Vertex>(66);
    auto v67 = std::make_shared<Vertex>(67);
    auto v68 = std::make_shared<Vertex>(68);
    auto v69 = std::make_shared<Vertex>(69);
    auto v70 = std::make_shared<Vertex>(70);
    auto v71 = std::make_shared<Vertex>(71);
    auto v72 = std::make_shared<Vertex>(72);
    auto v73 = std::make_shared<Vertex>(73);
    auto v74 = std::make_shared<Vertex>(74);
    auto v75 = std::make_shared<Vertex>(75);
    auto v76 = std::make_shared<Vertex>(76);
    auto v77 = std::make_shared<Vertex>(77);
    auto v78 = std::make_shared<Vertex>(78);
    auto v79 = std::make_shared<Vertex>(79);
    auto v80 = std::make_shared<Vertex>(80);
    auto v81 = std::make_shared<Vertex>(81);
    auto v82 = std::make_shared<Vertex>(82);
    auto v83 = std::make_shared<Vertex>(83);
    auto v84 = std::make_shared<Vertex>(84);
    auto v85 = std::make_shared<Vertex>(85);
    std::vector<std::shared_ptr<Edge>> v0edges = {
        std::make_shared<Edge>(v0, v60, 97.316864),
    };
    v0->setEdges(v0edges);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v84, 97.316864),
    };
    v1->setEdges(v1edges);
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v72, 97.316864),
    };
    v2->setEdges(v2edges);
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v85, 97.316864),
    };
    v3->setEdges(v3edges);
    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v42, 44.262295),
        std::make_shared<Edge>(v4, v84, 40.983601),
    };
    v4->setEdges(v4edges);
    std::vector<std::shared_ptr<Edge>> v5edges = {
        std::make_shared<Edge>(v5, v42, 40.983604),
        std::make_shared<Edge>(v5, v44, 44.262291),
    };
    v5->setEdges(v5edges);
    std::vector<std::shared_ptr<Edge>> v6edges = {
        std::make_shared<Edge>(v6, v44, 40.983604),
        std::make_shared<Edge>(v6, v60, 44.262291),
    };
    v6->setEdges(v6edges);
    std::vector<std::shared_ptr<Edge>> v7edges = {
        std::make_shared<Edge>(v7, v8, 85.245895),
        std::make_shared<Edge>(v7, v43, 40.983601),
        std::make_shared<Edge>(v7, v45, 44.262295),
    };
    v7->setEdges(v7edges);
    std::vector<std::shared_ptr<Edge>> v8edges = {
        std::make_shared<Edge>(v8, v7, 85.245895),
        std::make_shared<Edge>(v8, v9, 85.245895),
        std::make_shared<Edge>(v8, v45, 40.983604),
        std::make_shared<Edge>(v8, v61, 44.262291),
    };
    v8->setEdges(v8edges);
    std::vector<std::shared_ptr<Edge>> v9edges = {
        std::make_shared<Edge>(v9, v8, 85.245895),
        std::make_shared<Edge>(v9, v61, 40.983604),
        std::make_shared<Edge>(v9, v78, 44.262291),
    };
    v9->setEdges(v9edges);
    std::vector<std::shared_ptr<Edge>> v10edges = {
        std::make_shared<Edge>(v10, v76, 40.983601),
        std::make_shared<Edge>(v10, v77, 44.262295),
    };
    v10->setEdges(v10edges);
    std::vector<std::shared_ptr<Edge>> v11edges = {
        std::make_shared<Edge>(v11, v77, 40.983604),
        std::make_shared<Edge>(v11, v79, 44.262291),
    };
    v11->setEdges(v11edges);
    std::vector<std::shared_ptr<Edge>> v12edges = {
        std::make_shared<Edge>(v12, v79, 40.983604),
        std::make_shared<Edge>(v12, v82, 44.262291),
    };
    v12->setEdges(v12edges);
    std::vector<std::shared_ptr<Edge>> v13edges = {
        std::make_shared<Edge>(v13, v14, 85.245895),
        std::make_shared<Edge>(v13, v80, 40.983601),
        std::make_shared<Edge>(v13, v81, 44.262295),
    };
    v13->setEdges(v13edges);
    std::vector<std::shared_ptr<Edge>> v14edges = {
        std::make_shared<Edge>(v14, v13, 85.245895),
        std::make_shared<Edge>(v14, v15, 85.245895),
        std::make_shared<Edge>(v14, v46, 44.262291),
        std::make_shared<Edge>(v14, v81, 40.983604),
    };
    v14->setEdges(v14edges);
    std::vector<std::shared_ptr<Edge>> v15edges = {
        std::make_shared<Edge>(v15, v14, 85.245895),
        std::make_shared<Edge>(v15, v46, 40.983604),
        std::make_shared<Edge>(v15, v48, 44.262291),
    };
    v15->setEdges(v15edges);
    std::vector<std::shared_ptr<Edge>> v16edges = {
        std::make_shared<Edge>(v16, v47, 44.262295),
        std::make_shared<Edge>(v16, v83, 40.983601),
    };
    v16->setEdges(v16edges);
    std::vector<std::shared_ptr<Edge>> v17edges = {
        std::make_shared<Edge>(v17, v47, 40.983604),
        std::make_shared<Edge>(v17, v49, 44.262291),
    };
    v17->setEdges(v17edges);
    std::vector<std::shared_ptr<Edge>> v18edges = {
        std::make_shared<Edge>(v18, v49, 40.983604),
        std::make_shared<Edge>(v18, v52, 44.262291),
    };
    v18->setEdges(v18edges);
    std::vector<std::shared_ptr<Edge>> v19edges = {
        std::make_shared<Edge>(v19, v20, 85.245895),
        std::make_shared<Edge>(v19, v50, 40.983601),
        std::make_shared<Edge>(v19, v51, 44.262295),
    };
    v19->setEdges(v19edges);
    std::vector<std::shared_ptr<Edge>> v20edges = {
        std::make_shared<Edge>(v20, v19, 85.245895),
        std::make_shared<Edge>(v20, v21, 85.245895),
        std::make_shared<Edge>(v20, v51, 40.983604),
        std::make_shared<Edge>(v20, v53, 44.262291),
    };
    v20->setEdges(v20edges);
    std::vector<std::shared_ptr<Edge>> v21edges = {
        std::make_shared<Edge>(v21, v20, 85.245895),
        std::make_shared<Edge>(v21, v53, 40.983604),
        std::make_shared<Edge>(v21, v56, 44.262291),
    };
    v21->setEdges(v21edges);
    std::vector<std::shared_ptr<Edge>> v22edges = {
        std::make_shared<Edge>(v22, v54, 40.983601),
        std::make_shared<Edge>(v22, v55, 44.262295),
    };
    v22->setEdges(v22edges);
    std::vector<std::shared_ptr<Edge>> v23edges = {
        std::make_shared<Edge>(v23, v55, 40.983604),
        std::make_shared<Edge>(v23, v57, 44.262291),
    };
    v23->setEdges(v23edges);
    std::vector<std::shared_ptr<Edge>> v24edges = {
        std::make_shared<Edge>(v24, v57, 40.983604),
        std::make_shared<Edge>(v24, v62, 44.262291),
    };
    v24->setEdges(v24edges);
    std::vector<std::shared_ptr<Edge>> v25edges = {
        std::make_shared<Edge>(v25, v26, 85.245895),
        std::make_shared<Edge>(v25, v58, 40.983601),
        std::make_shared<Edge>(v25, v59, 44.262295),
    };
    v25->setEdges(v25edges);
    std::vector<std::shared_ptr<Edge>> v26edges = {
        std::make_shared<Edge>(v26, v25, 85.245895),
        std::make_shared<Edge>(v26, v27, 85.245895),
        std::make_shared<Edge>(v26, v59, 40.983604),
        std::make_shared<Edge>(v26, v63, 44.262291),
    };
    v26->setEdges(v26edges);
    std::vector<std::shared_ptr<Edge>> v27edges = {
        std::make_shared<Edge>(v27, v26, 85.245895),
        std::make_shared<Edge>(v27, v63, 40.983604),
        std::make_shared<Edge>(v27, v66, 44.262291),
    };
    v27->setEdges(v27edges);
    std::vector<std::shared_ptr<Edge>> v28edges = {
        std::make_shared<Edge>(v28, v64, 40.983601),
        std::make_shared<Edge>(v28, v65, 44.262295),
    };
    v28->setEdges(v28edges);
    std::vector<std::shared_ptr<Edge>> v29edges = {
        std::make_shared<Edge>(v29, v65, 40.983604),
        std::make_shared<Edge>(v29, v67, 44.262291),
    };
    v29->setEdges(v29edges);
    std::vector<std::shared_ptr<Edge>> v30edges = {
        std::make_shared<Edge>(v30, v67, 40.983604),
        std::make_shared<Edge>(v30, v70, 44.262291),
    };
    v30->setEdges(v30edges);
    std::vector<std::shared_ptr<Edge>> v31edges = {
        std::make_shared<Edge>(v31, v32, 85.245895),
        std::make_shared<Edge>(v31, v68, 40.983601),
        std::make_shared<Edge>(v31, v69, 44.262295),
    };
    v31->setEdges(v31edges);
    std::vector<std::shared_ptr<Edge>> v32edges = {
        std::make_shared<Edge>(v32, v31, 85.245895),
        std::make_shared<Edge>(v32, v33, 85.245895),
        std::make_shared<Edge>(v32, v69, 40.983604),
        std::make_shared<Edge>(v32, v71, 44.262291),
    };
    v32->setEdges(v32edges);
    std::vector<std::shared_ptr<Edge>> v33edges = {
        std::make_shared<Edge>(v33, v32, 85.245895),
        std::make_shared<Edge>(v33, v71, 40.983604),
        std::make_shared<Edge>(v33, v74, 44.262291),
    };
    v33->setEdges(v33edges);
    std::vector<std::shared_ptr<Edge>> v34edges = {
        std::make_shared<Edge>(v34, v72, 40.983601),
        std::make_shared<Edge>(v34, v73, 44.262295),
    };
    v34->setEdges(v34edges);
    std::vector<std::shared_ptr<Edge>> v35edges = {
        std::make_shared<Edge>(v35, v73, 40.983604),
        std::make_shared<Edge>(v35, v75, 44.262291),
    };
    v35->setEdges(v35edges);
    std::vector<std::shared_ptr<Edge>> v36edges = {
        std::make_shared<Edge>(v36, v75, 40.983604),
        std::make_shared<Edge>(v36, v85, 44.262291),
    };
    v36->setEdges(v36edges);
    std::vector<std::shared_ptr<Edge>> v37edges = {
        std::make_shared<Edge>(v37, v38, 42.622948),
        std::make_shared<Edge>(v37, v39, 85.245895),
    };
    v37->setEdges(v37edges);
    std::vector<std::shared_ptr<Edge>> v38edges = {
        std::make_shared<Edge>(v38, v37, 42.622948),
        std::make_shared<Edge>(v38, v39, 42.622948),
        std::make_shared<Edge>(v38, v40, 85.245895),
        std::make_shared<Edge>(v38, v42, 65.573769),
    };
    v38->setEdges(v38edges);
    std::vector<std::shared_ptr<Edge>> v39edges = {
        std::make_shared<Edge>(v39, v37, 85.245895),
        std::make_shared<Edge>(v39, v38, 42.622948),
        std::make_shared<Edge>(v39, v40, 42.622948),
        std::make_shared<Edge>(v39, v41, 85.245895),
    };
    v39->setEdges(v39edges);
    std::vector<std::shared_ptr<Edge>> v40edges = {
        std::make_shared<Edge>(v40, v38, 85.245895),
        std::make_shared<Edge>(v40, v39, 42.622948),
        std::make_shared<Edge>(v40, v41, 42.622948),
        std::make_shared<Edge>(v40, v44, 65.573769),
    };
    v40->setEdges(v40edges);
    std::vector<std::shared_ptr<Edge>> v41edges = {
        std::make_shared<Edge>(v41, v39, 85.245895),
        std::make_shared<Edge>(v41, v40, 42.622948),
    };
    v41->setEdges(v41edges);
    std::vector<std::shared_ptr<Edge>> v42edges = {
        std::make_shared<Edge>(v42, v4, 44.262295),
        std::make_shared<Edge>(v42, v5, 40.983604),
        std::make_shared<Edge>(v42, v38, 65.573769),
        std::make_shared<Edge>(v42, v44, 85.245895),
        std::make_shared<Edge>(v42, v45, 19.672132),
        std::make_shared<Edge>(v42, v84, 85.245895),
    };
    v42->setEdges(v42edges);
    std::vector<std::shared_ptr<Edge>> v43edges = {
        std::make_shared<Edge>(v43, v7, 40.983601),
        std::make_shared<Edge>(v43, v45, 85.245895),
        std::make_shared<Edge>(v43, v76, 19.672136),
        std::make_shared<Edge>(v43, v84, 19.672129),
    };
    v43->setEdges(v43edges);
    std::vector<std::shared_ptr<Edge>> v44edges = {
        std::make_shared<Edge>(v44, v5, 44.262291),
        std::make_shared<Edge>(v44, v6, 40.983604),
        std::make_shared<Edge>(v44, v40, 65.573769),
        std::make_shared<Edge>(v44, v42, 85.245895),
        std::make_shared<Edge>(v44, v60, 85.245895),
        std::make_shared<Edge>(v44, v61, 19.672132),
    };
    v44->setEdges(v44edges);
    std::vector<std::shared_ptr<Edge>> v45edges = {
        std::make_shared<Edge>(v45, v7, 44.262295),
        std::make_shared<Edge>(v45, v8, 40.983604),
        std::make_shared<Edge>(v45, v42, 19.672132),
        std::make_shared<Edge>(v45, v43, 85.245895),
        std::make_shared<Edge>(v45, v61, 85.245895),
        std::make_shared<Edge>(v45, v77, 19.672132),
    };
    v45->setEdges(v45edges);
    std::vector<std::shared_ptr<Edge>> v46edges = {
        std::make_shared<Edge>(v46, v14, 44.262291),
        std::make_shared<Edge>(v46, v15, 40.983604),
        std::make_shared<Edge>(v46, v48, 85.245895),
        std::make_shared<Edge>(v46, v49, 19.672132),
        std::make_shared<Edge>(v46, v79, 19.672131),
        std::make_shared<Edge>(v46, v81, 85.245895),
    };
    v46->setEdges(v46edges);
    std::vector<std::shared_ptr<Edge>> v47edges = {
        std::make_shared<Edge>(v47, v16, 44.262295),
        std::make_shared<Edge>(v47, v17, 40.983604),
        std::make_shared<Edge>(v47, v49, 85.245895),
        std::make_shared<Edge>(v47, v51, 19.672132),
        std::make_shared<Edge>(v47, v81, 19.672132),
        std::make_shared<Edge>(v47, v83, 85.245895),
    };
    v47->setEdges(v47edges);
    std::vector<std::shared_ptr<Edge>> v48edges = {
        std::make_shared<Edge>(v48, v15, 44.262291),
        std::make_shared<Edge>(v48, v46, 85.245895),
        std::make_shared<Edge>(v48, v52, 19.672129),
        std::make_shared<Edge>(v48, v82, 19.672132),
    };
    v48->setEdges(v48edges);
    std::vector<std::shared_ptr<Edge>> v49edges = {
        std::make_shared<Edge>(v49, v17, 44.262291),
        std::make_shared<Edge>(v49, v18, 40.983604),
        std::make_shared<Edge>(v49, v46, 19.672132),
        std::make_shared<Edge>(v49, v47, 85.245895),
        std::make_shared<Edge>(v49, v52, 85.245895),
        std::make_shared<Edge>(v49, v53, 19.672132),
    };
    v49->setEdges(v49edges);
    std::vector<std::shared_ptr<Edge>> v50edges = {
        std::make_shared<Edge>(v50, v19, 40.983601),
        std::make_shared<Edge>(v50, v51, 85.245895),
        std::make_shared<Edge>(v50, v54, 19.672132),
        std::make_shared<Edge>(v50, v83, 19.672131),
    };
    v50->setEdges(v50edges);
    std::vector<std::shared_ptr<Edge>> v51edges = {
        std::make_shared<Edge>(v51, v19, 44.262295),
        std::make_shared<Edge>(v51, v20, 40.983604),
        std::make_shared<Edge>(v51, v47, 19.672132),
        std::make_shared<Edge>(v51, v50, 85.245895),
        std::make_shared<Edge>(v51, v53, 85.245895),
        std::make_shared<Edge>(v51, v55, 19.672132),
    };
    v51->setEdges(v51edges);
    std::vector<std::shared_ptr<Edge>> v52edges = {
        std::make_shared<Edge>(v52, v18, 44.262291),
        std::make_shared<Edge>(v52, v48, 19.672129),
        std::make_shared<Edge>(v52, v49, 85.245895),
        std::make_shared<Edge>(v52, v56, 19.672132),
    };
    v52->setEdges(v52edges);
    std::vector<std::shared_ptr<Edge>> v53edges = {
        std::make_shared<Edge>(v53, v20, 44.262291),
        std::make_shared<Edge>(v53, v21, 40.983604),
        std::make_shared<Edge>(v53, v49, 19.672132),
        std::make_shared<Edge>(v53, v51, 85.245895),
        std::make_shared<Edge>(v53, v56, 85.245895),
        std::make_shared<Edge>(v53, v57, 19.672132),
    };
    v53->setEdges(v53edges);
    std::vector<std::shared_ptr<Edge>> v54edges = {
        std::make_shared<Edge>(v54, v22, 40.983601),
        std::make_shared<Edge>(v54, v50, 19.672132),
        std::make_shared<Edge>(v54, v55, 85.245895),
        std::make_shared<Edge>(v54, v58, 19.672129),
    };
    v54->setEdges(v54edges);
    std::vector<std::shared_ptr<Edge>> v55edges = {
        std::make_shared<Edge>(v55, v22, 44.262295),
        std::make_shared<Edge>(v55, v23, 40.983604),
        std::make_shared<Edge>(v55, v51, 19.672132),
        std::make_shared<Edge>(v55, v54, 85.245895),
        std::make_shared<Edge>(v55, v57, 85.245895),
        std::make_shared<Edge>(v55, v59, 19.672132),
    };
    v55->setEdges(v55edges);
    std::vector<std::shared_ptr<Edge>> v56edges = {
        std::make_shared<Edge>(v56, v21, 44.262291),
        std::make_shared<Edge>(v56, v52, 19.672132),
        std::make_shared<Edge>(v56, v53, 85.245895),
        std::make_shared<Edge>(v56, v62, 19.672131),
    };
    v56->setEdges(v56edges);
    std::vector<std::shared_ptr<Edge>> v57edges = {
        std::make_shared<Edge>(v57, v23, 44.262291),
        std::make_shared<Edge>(v57, v24, 40.983604),
        std::make_shared<Edge>(v57, v53, 19.672132),
        std::make_shared<Edge>(v57, v55, 85.245895),
        std::make_shared<Edge>(v57, v62, 85.245895),
        std::make_shared<Edge>(v57, v63, 19.672132),
    };
    v57->setEdges(v57edges);
    std::vector<std::shared_ptr<Edge>> v58edges = {
        std::make_shared<Edge>(v58, v25, 40.983601),
        std::make_shared<Edge>(v58, v54, 19.672129),
        std::make_shared<Edge>(v58, v59, 85.245895),
        std::make_shared<Edge>(v58, v64, 19.672132),
    };
    v58->setEdges(v58edges);
    std::vector<std::shared_ptr<Edge>> v59edges = {
        std::make_shared<Edge>(v59, v25, 44.262295),
        std::make_shared<Edge>(v59, v26, 40.983604),
        std::make_shared<Edge>(v59, v55, 19.672132),
        std::make_shared<Edge>(v59, v58, 85.245895),
        std::make_shared<Edge>(v59, v63, 85.245895),
        std::make_shared<Edge>(v59, v65, 19.672131),
    };
    v59->setEdges(v59edges);
    std::vector<std::shared_ptr<Edge>> v60edges = {
        std::make_shared<Edge>(v60, v0, 97.316864),
        std::make_shared<Edge>(v60, v6, 44.262291),
        std::make_shared<Edge>(v60, v44, 85.245895),
        std::make_shared<Edge>(v60, v78, 19.672136),
    };
    v60->setEdges(v60edges);
    std::vector<std::shared_ptr<Edge>> v61edges = {
        std::make_shared<Edge>(v61, v8, 44.262291),
        std::make_shared<Edge>(v61, v9, 40.983604),
        std::make_shared<Edge>(v61, v44, 19.672132),
        std::make_shared<Edge>(v61, v45, 85.245895),
        std::make_shared<Edge>(v61, v78, 85.245895),
        std::make_shared<Edge>(v61, v79, 19.672132),
    };
    v61->setEdges(v61edges);
    std::vector<std::shared_ptr<Edge>> v62edges = {
        std::make_shared<Edge>(v62, v24, 44.262291),
        std::make_shared<Edge>(v62, v56, 19.672131),
        std::make_shared<Edge>(v62, v57, 85.245895),
        std::make_shared<Edge>(v62, v66, 19.672134),
    };
    v62->setEdges(v62edges);
    std::vector<std::shared_ptr<Edge>> v63edges = {
        std::make_shared<Edge>(v63, v26, 44.262291),
        std::make_shared<Edge>(v63, v27, 40.983604),
        std::make_shared<Edge>(v63, v57, 19.672132),
        std::make_shared<Edge>(v63, v59, 85.245895),
        std::make_shared<Edge>(v63, v66, 85.245895),
        std::make_shared<Edge>(v63, v67, 19.672131),
    };
    v63->setEdges(v63edges);
    std::vector<std::shared_ptr<Edge>> v64edges = {
        std::make_shared<Edge>(v64, v28, 40.983601),
        std::make_shared<Edge>(v64, v58, 19.672132),
        std::make_shared<Edge>(v64, v65, 85.245895),
        std::make_shared<Edge>(v64, v68, 19.672129),
    };
    v64->setEdges(v64edges);
    std::vector<std::shared_ptr<Edge>> v65edges = {
        std::make_shared<Edge>(v65, v28, 44.262295),
        std::make_shared<Edge>(v65, v29, 40.983604),
        std::make_shared<Edge>(v65, v59, 19.672131),
        std::make_shared<Edge>(v65, v64, 85.245895),
        std::make_shared<Edge>(v65, v67, 85.245895),
        std::make_shared<Edge>(v65, v69, 19.672132),
    };
    v65->setEdges(v65edges);
    std::vector<std::shared_ptr<Edge>> v66edges = {
        std::make_shared<Edge>(v66, v27, 44.262291),
        std::make_shared<Edge>(v66, v62, 19.672134),
        std::make_shared<Edge>(v66, v63, 85.245895),
        std::make_shared<Edge>(v66, v70, 19.672129),
    };
    v66->setEdges(v66edges);
    std::vector<std::shared_ptr<Edge>> v67edges = {
        std::make_shared<Edge>(v67, v29, 44.262291),
        std::make_shared<Edge>(v67, v30, 40.983604),
        std::make_shared<Edge>(v67, v63, 19.672131),
        std::make_shared<Edge>(v67, v65, 85.245895),
        std::make_shared<Edge>(v67, v70, 85.245895),
        std::make_shared<Edge>(v67, v71, 19.672132),
    };
    v67->setEdges(v67edges);
    std::vector<std::shared_ptr<Edge>> v68edges = {
        std::make_shared<Edge>(v68, v31, 40.983601),
        std::make_shared<Edge>(v68, v64, 19.672129),
        std::make_shared<Edge>(v68, v69, 85.245895),
        std::make_shared<Edge>(v68, v72, 19.672136),
    };
    v68->setEdges(v68edges);
    std::vector<std::shared_ptr<Edge>> v69edges = {
        std::make_shared<Edge>(v69, v31, 44.262295),
        std::make_shared<Edge>(v69, v32, 40.983604),
        std::make_shared<Edge>(v69, v65, 19.672132),
        std::make_shared<Edge>(v69, v68, 85.245895),
        std::make_shared<Edge>(v69, v71, 85.245895),
        std::make_shared<Edge>(v69, v73, 19.672132),
    };
    v69->setEdges(v69edges);
    std::vector<std::shared_ptr<Edge>> v70edges = {
        std::make_shared<Edge>(v70, v30, 44.262291),
        std::make_shared<Edge>(v70, v66, 19.672129),
        std::make_shared<Edge>(v70, v67, 85.245895),
        std::make_shared<Edge>(v70, v74, 19.672136),
    };
    v70->setEdges(v70edges);
    std::vector<std::shared_ptr<Edge>> v71edges = {
        std::make_shared<Edge>(v71, v32, 44.262291),
        std::make_shared<Edge>(v71, v33, 40.983604),
        std::make_shared<Edge>(v71, v67, 19.672132),
        std::make_shared<Edge>(v71, v69, 85.245895),
        std::make_shared<Edge>(v71, v74, 85.245895),
        std::make_shared<Edge>(v71, v75, 19.672132),
    };
    v71->setEdges(v71edges);
    std::vector<std::shared_ptr<Edge>> v72edges = {
        std::make_shared<Edge>(v72, v2, 97.316864),
        std::make_shared<Edge>(v72, v34, 40.983601),
        std::make_shared<Edge>(v72, v68, 19.672136),
        std::make_shared<Edge>(v72, v73, 85.245895),
    };
    v72->setEdges(v72edges);
    std::vector<std::shared_ptr<Edge>> v73edges = {
        std::make_shared<Edge>(v73, v34, 44.262295),
        std::make_shared<Edge>(v73, v35, 40.983604),
        std::make_shared<Edge>(v73, v69, 19.672132),
        std::make_shared<Edge>(v73, v72, 85.245895),
        std::make_shared<Edge>(v73, v75, 85.245895),
    };
    v73->setEdges(v73edges);
    std::vector<std::shared_ptr<Edge>> v74edges = {
        std::make_shared<Edge>(v74, v33, 44.262291),
        std::make_shared<Edge>(v74, v70, 19.672136),
        std::make_shared<Edge>(v74, v71, 85.245895),
        std::make_shared<Edge>(v74, v85, 19.672129),
    };
    v74->setEdges(v74edges);
    std::vector<std::shared_ptr<Edge>> v75edges = {
        std::make_shared<Edge>(v75, v35, 44.262291),
        std::make_shared<Edge>(v75, v36, 40.983604),
        std::make_shared<Edge>(v75, v71, 19.672132),
        std::make_shared<Edge>(v75, v73, 85.245895),
        std::make_shared<Edge>(v75, v85, 85.245895),
    };
    v75->setEdges(v75edges);
    std::vector<std::shared_ptr<Edge>> v76edges = {
        std::make_shared<Edge>(v76, v10, 40.983601),
        std::make_shared<Edge>(v76, v43, 19.672136),
        std::make_shared<Edge>(v76, v77, 85.245895),
        std::make_shared<Edge>(v76, v80, 19.672129),
    };
    v76->setEdges(v76edges);
    std::vector<std::shared_ptr<Edge>> v77edges = {
        std::make_shared<Edge>(v77, v10, 44.262295),
        std::make_shared<Edge>(v77, v11, 40.983604),
        std::make_shared<Edge>(v77, v45, 19.672132),
        std::make_shared<Edge>(v77, v76, 85.245895),
        std::make_shared<Edge>(v77, v79, 85.245895),
        std::make_shared<Edge>(v77, v81, 19.672131),
    };
    v77->setEdges(v77edges);
    std::vector<std::shared_ptr<Edge>> v78edges = {
        std::make_shared<Edge>(v78, v9, 44.262291),
        std::make_shared<Edge>(v78, v60, 19.672136),
        std::make_shared<Edge>(v78, v61, 85.245895),
        std::make_shared<Edge>(v78, v82, 19.672129),
    };
    v78->setEdges(v78edges);
    std::vector<std::shared_ptr<Edge>> v79edges = {
        std::make_shared<Edge>(v79, v11, 44.262291),
        std::make_shared<Edge>(v79, v12, 40.983604),
        std::make_shared<Edge>(v79, v46, 19.672131),
        std::make_shared<Edge>(v79, v61, 19.672132),
        std::make_shared<Edge>(v79, v77, 85.245895),
        std::make_shared<Edge>(v79, v82, 85.245895),
    };
    v79->setEdges(v79edges);
    std::vector<std::shared_ptr<Edge>> v80edges = {
        std::make_shared<Edge>(v80, v13, 40.983601),
        std::make_shared<Edge>(v80, v76, 19.672129),
        std::make_shared<Edge>(v80, v81, 85.245895),
        std::make_shared<Edge>(v80, v83, 19.672134),
    };
    v80->setEdges(v80edges);
    std::vector<std::shared_ptr<Edge>> v81edges = {
        std::make_shared<Edge>(v81, v13, 44.262295),
        std::make_shared<Edge>(v81, v14, 40.983604),
        std::make_shared<Edge>(v81, v46, 85.245895),
        std::make_shared<Edge>(v81, v47, 19.672132),
        std::make_shared<Edge>(v81, v77, 19.672131),
        std::make_shared<Edge>(v81, v80, 85.245895),
    };
    v81->setEdges(v81edges);
    std::vector<std::shared_ptr<Edge>> v82edges = {
        std::make_shared<Edge>(v82, v12, 44.262291),
        std::make_shared<Edge>(v82, v48, 19.672132),
        std::make_shared<Edge>(v82, v78, 19.672129),
        std::make_shared<Edge>(v82, v79, 85.245895),
    };
    v82->setEdges(v82edges);
    std::vector<std::shared_ptr<Edge>> v83edges = {
        std::make_shared<Edge>(v83, v16, 40.983601),
        std::make_shared<Edge>(v83, v47, 85.245895),
        std::make_shared<Edge>(v83, v50, 19.672131),
        std::make_shared<Edge>(v83, v80, 19.672134),
    };
    v83->setEdges(v83edges);
    std::vector<std::shared_ptr<Edge>> v84edges = {
        std::make_shared<Edge>(v84, v1, 97.316864),
        std::make_shared<Edge>(v84, v4, 40.983601),
        std::make_shared<Edge>(v84, v42, 85.245895),
        std::make_shared<Edge>(v84, v43, 19.672129),
    };
    v84->setEdges(v84edges);
    std::vector<std::shared_ptr<Edge>> v85edges = {
        std::make_shared<Edge>(v85, v3, 97.316864),
        std::make_shared<Edge>(v85, v36, 44.262291),
        std::make_shared<Edge>(v85, v74, 19.672129),
        std::make_shared<Edge>(v85, v75, 85.245895),
    };
    v85->setEdges(v85edges);
    std::vector<std::shared_ptr<Vertex>> vertices = {
        v0,
        v1,
        v2,
        v3,
        v4,
        v5,
        v6,
        v7,
        v8,
        v9,
        v10,
        v11,
        v12,
        v13,
        v14,
        v15,
        v16,
        v17,
        v18,
        v19,
        v20,
        v21,
        v22,
        v23,
        v24,
        v25,
        v26,
        v27,
        v28,
        v29,
        v30,
        v31,
        v32,
        v33,
        v34,
        v35,
        v36,
        v37,
        v38,
        v39,
        v40,
        v41,
        v42,
        v43,
        v44,
        v45,
        v46,
        v47,
        v48,
        v49,
        v50,
        v51,
        v52,
        v53,
        v54,
        v55,
        v56,
        v57,
        v58,
        v59,
        v60,
        v61,
        v62,
        v63,
        v64,
        v65,
        v66,
        v67,
        v68,
        v69,
        v70,
        v71,
        v72,
        v73,
        v74,
        v75,
        v76,
        v77,
        v78,
        v79,
        v80,
        v81,
        v82,
        v83,
        v84,
        v85
    };
    return std::make_shared<Graph>(vertices);
}