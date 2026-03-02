#include "models/models.h"

#include <iterator>
#include <utility>

namespace mapf::models {

AgentState::AgentState(AgentId agent_id, graph::NodeId node_id) :
    agent_id{std::move(agent_id)}, node_id{std::move(node_id)} {}

AgentState::AgentState(proto::AgentState agent_state_proto) :
    agent_id{std::move(agent_state_proto.agent_id())},
    node_id{std::move(agent_state_proto.node_id())} {}

AgentTask::AgentTask(AgentId agent_id, graph::Endpoints endpoints) :
    agent_id{std::move(agent_id)}, endpoints{std::move(endpoints)} {}

AgentTask::AgentTask(proto::AgentTask agent_task_proto) :
    agent_id{std::move(agent_task_proto.agent_id())},
    endpoints{std::move(agent_task_proto.endpoints())} {}

AgentPath::AgentPath(AgentId agent_id, graph::NodeIdsList path) :
    agent_id{std::move(agent_id)}, path{std::move(path)} {}

AgentPath::AgentPath(proto::AgentPath agent_path_proto) :
    agent_id{std::move(agent_path_proto.agent_id())},
    path{
        std::make_move_iterator(agent_path_proto.path().begin()),
        std::make_move_iterator(agent_path_proto.path().end())} {}

MAPFProblem::MAPFProblem(graph::Graph graph, AgentTasks agent_tasks) :
    graph{std::move(graph)}, agent_tasks{std::move(agent_tasks)} {}

MAPFProblem::MAPFProblem(proto::MAPFProblem mapf_problem_proto) :
    graph{std::move(mapf_problem_proto.graph())} {
    agent_tasks.reserve(mapf_problem_proto.agent_tasks().size());
    for (auto&& agent_task : mapf_problem_proto.agent_tasks()) {
        agent_tasks.emplace(agent_task.agent_id(), std::move(agent_task));
    }
}

MAPFSolution::MAPFSolution(AgentPaths agent_paths) : agent_paths{std::move(agent_paths)} {}

MAPFSolution::MAPFSolution(proto::MAPFSolution mapf_solution_proto) {
    agent_paths.reserve(mapf_solution_proto.agent_paths().size());
    for (auto&& agent_path : mapf_solution_proto.agent_paths()) {
        agent_paths.emplace(agent_path.agent_id(), std::move(agent_path));
    }
}

}  // namespace mapf::models