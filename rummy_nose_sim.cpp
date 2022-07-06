#include "rummy_simulation.hpp"

#include "stat/position_stat.hpp"
#include "stat/velocity_stat.hpp"
#include "stat/kick_counter.hpp"

#include <iostream>

struct RummySimuParams : public simu::simulation::defaults::RummySimuParams {
};

int main()
{
    using namespace simu;
    using namespace simulation;

    RummySimuParams params;
    params.sim_time = 500001;
    RummySimulation sim(params);

    // sim.add_stat(std::make_shared<stat::PositionStat>())
    //     .add_stat(std::make_shared<stat::VelocityStat>())
    //     .add_stat(std::make_shared<stat::KickCounter>());
    sim.add_stat(std::make_shared<stat::PositionStat>());

    sim.spin();

    return 0;
}