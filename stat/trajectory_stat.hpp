#ifndef TRAJECTORY_STAT_HPP
#define TRAJECTORY_STAT_HPP

#include <rummy_simulation.hpp>
#include <stat/stat_base.hpp>

namespace simu {
    namespace stat {
        using namespace simulation;

        class TrajectoryStat : public StatBase {
        public:
            void operator()(const SimulationPtr sim) override
            {
                using namespace simulation;
                auto fsim = std::static_pointer_cast<RummySimulation>(sim);

                if (!fsim->stats_enabled())
                    return;

                _create_log_file(fsim, "trajectory.dat");

                if (fsim->is_kicking()) {
                    return;
                }

                *_log_file << fsim->iteration() << " " << fsim->iteration() * fsim->params().timestep << " ";
                for (const RummyIndividualPtr& f : fsim->fish())
                    *_log_file << f->traj_pose().x / fsim->params().radius << " " << f->traj_pose().y / fsim->params().radius << " ";
                *_log_file << std::endl;
            }
        };

    } // namespace stat
} // namespace simu

#endif
