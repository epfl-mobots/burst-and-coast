#ifndef VELOCITY_STAT_HPP
#define VELOCITY_STAT_HPP

#include <rummy_simulation.hpp>
#include <stat/stat_base.hpp>

namespace simu {
    namespace stat {
        using namespace simulation;

        class VelocityStat : public StatBase {
        public:
            void operator()(const SimulationPtr sim) override
            {
                using namespace simulation;
                auto fsim = std::static_pointer_cast<RummySimulation>(sim);

                if (!fsim->stats_enabled())
                    return;

                _create_log_file(fsim, "velocities.dat");

                // if (fsim->iteration() == 0)
                //     *_log_file << "#iteration fish1_position fish2_position ..." << std::endl;

                *_log_file << fsim->iteration() << " " << fsim->kicking_idx() << " ";
                for (const RummyIndividualPtr& f : fsim->fish())
                    *_log_file << f->desired_speed().vx << " " << f->desired_speed().vy << " ";
                *_log_file << std::endl;
            }
        };

    } // namespace stat
} // namespace simu

#endif
