#include "rummy_simulation.hpp"

#include "stat/trajectory_stat.hpp"
#include "stat/clean_trajectory_stat.hpp"
#include "stat/kick_stat.hpp"
#include "stat/speed_stat.hpp"
#include "stat/pose_stat.hpp"

#include <iostream>

struct RummySimuParams : public simu::simulation::defaults::RummySimuParams {
};

struct RummyIndividualParams : public simu::simulation::defaults::RummyIndividualParams {
};

#define NUM_FISH 2

int main()
{
    using namespace simu;
    using namespace simulation;

    // init simu
    RummySimuParams params;
    params.sim_time = 150000;
    // params.timestep = 0.12;
    params.timestep = 0.1;
    params.num_fish = NUM_FISH;
    RummySimulation sim(params);

    // init indu params
    RummyIndividualParams iparams;
    if (NUM_FISH == 1) {
        // interactions
        iparams.perceived_agents = 0;
        iparams.perceived_agents = 1;
        iparams.gamma_rand = 0.3;
        iparams.gamma_wall = 0.23;
        iparams.gamma_sym = 0.;
        iparams.gamma_asym = 0.;

        iparams.dw = 6.;
        iparams.dc = 0.7;
        iparams.alpha_w = 0.;
        iparams.gamma_attraction = 0.3;
        iparams.gamma_alignment = 0.3;

        // kicks
        iparams.vmean = 43.;
        iparams.vmin = 1.;
        iparams.vmem = 0.9;
        iparams.vmem12 = 0.0;
        iparams.vcut = 30.;
        iparams.taumean = 0.525;
        iparams.taumin = 0.2;
        iparams.tau0 = 0.8;
    }
    else if (NUM_FISH == 2) {
        // interactions
        iparams.perceived_agents = 1;
        iparams.gamma_rand = 0.3;
        iparams.gamma_wall = 0.23;
        iparams.gamma_sym = 0.;
        iparams.gamma_asym = 0.;

        iparams.dw = 6.;
        iparams.dc = 0.7;
        iparams.alpha_w = 0.;
        iparams.gamma_attraction = 0.3;
        iparams.gamma_alignment = 0.3;

        // kicks
        iparams.vmean = 40.;
        iparams.vmin = 1.;
        iparams.vmem = 0.9;
        iparams.vmem12 = 0.5;
        iparams.vcut = 35.;
        iparams.taumean = 0.525;
        iparams.taumin = 0.2;
        iparams.tau0 = 0.8;
    }
    else if (NUM_FISH == 5) {
        // interactions
        iparams.perceived_agents = 1;
        iparams.gamma_rand = 0.3;
        iparams.gamma_wall = 0.23;
        iparams.gamma_sym = 0.;
        iparams.gamma_asym = 0.;

        iparams.dw = 6.;
        iparams.dc = 1.0;
        iparams.alpha_w = 0.;
        iparams.gamma_attraction = 0.37;
        iparams.gamma_alignment = 0.3;

        // kicks
        iparams.vmean = 43.;
        iparams.vmin = 1.;
        iparams.vmem = 0.9;
        iparams.vmem12 = 0.5;
        iparams.vcut = 34.;
        iparams.taumean = 0.525;
        iparams.taumin = 0.2;
        iparams.tau0 = 0.8;
    }
    else {
#ifndef NUM_FISH
#warning "NUM_FISH not defined"
#endif
        assert(false);
    }

    auto fish = sim.fish();
    for (auto f : fish) {
        f->params() = iparams;
    }

    // add stats
    sim.add_stat(std::make_shared<stat::TrajectoryStat>())
        .add_stat(std::make_shared<stat::CleanTrajectoryStat>())
        .add_stat(std::make_shared<stat::KickStat>())
        .add_stat(std::make_shared<stat::SpeedStat>())
        .add_stat(std::make_shared<stat::PoseStat>());

    sim.spin();

    return 0;
}