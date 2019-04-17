#include "rummy_simulation.hpp"
#include "stat/position_stat.hpp"
#include "stat/velocity_stat.hpp"

#include <iostream>

int main()
{
    using namespace simu;
    using namespace simulation;

    RummySimulation sim(true);

    sim.rummy_fish_settings().num_fish = 1;
    sim.rummy_fish_settings().gamma_rand = 0.35f;
    sim.rummy_fish_settings().gamma_wall = 0.12f;

    // sim.rummy_fish_settings().num_fish = 2;
    // sim.rummy_fish_settings().gamma_rand = 0.45f;
    // sim.rummy_fish_settings().gamma_wall = 0.12f;

    // sim.rummy_fish_settings().num_fish = 5;
    // sim.rummy_fish_settings().gamma_rand = 0.5f;
    // sim.rummy_fish_settings().gamma_wall = 0.15f;

    sim.reinit();
    sim.add_stat(std::make_shared<stat::PositionStat>())
        .add_stat(std::make_shared<stat::VelocityStat>());
    sim.spin();

    return 0;
}