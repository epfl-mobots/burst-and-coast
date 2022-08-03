#include "rummy_simulation.hpp"

#include "stat/trajectory_stat.hpp"
#include "stat/kick_stat.hpp"
#include "stat/speed_stat.hpp"
#include "stat/pose_stat.hpp"

#include <iostream>

struct RummySimuParams : public simu::simulation::defaults::RummySimuParams {
};

int main()
{
    using namespace simu;
    using namespace simulation;

    RummySimuParams params;
    params.sim_time = 500000;
    // params.timestep = 1 / 30.;
    // params.sim_time = 1500000;
    RummySimulation sim(params);

    sim.add_stat(std::make_shared<stat::TrajectoryStat>())
        .add_stat(std::make_shared<stat::KickStat>())
        .add_stat(std::make_shared<stat::SpeedStat>())
        .add_stat(std::make_shared<stat::PoseStat>());

    sim.spin();

    return 0;
}