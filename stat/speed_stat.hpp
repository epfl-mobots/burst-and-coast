#ifndef SPEED_STAT_HPP
#define SPEED_STAT_HPP

#include <stat/stat_base.hpp>
#include <rummy_simulation.hpp>

namespace simu {
    namespace stat {
        using namespace simulation;

        class SpeedStat : public StatBase {
        public:
            virtual void operator()(const std::shared_ptr<Simulation> sim) override
            {
                auto rsim = std::static_pointer_cast<RummySimulation>(sim);

                if (!rsim->stats_enabled())
                    return;

                _create_log_file(rsim, "speed.dat");

                if (rsim->is_kicking()) {
                    return;
                }

                *_log_file << rsim->iteration() << " " << rsim->iteration() * rsim->params().timestep << " ";
                for (const RummyIndividualPtr& f : rsim->fish())
                    *_log_file << f->traj_speed() / rsim->params().radius << " ";
                *_log_file << std::endl;
            }
        };

    } // namespace stat
} // namespace simu

#endif
