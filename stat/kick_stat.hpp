#ifndef KICK_STAT_HPP
#define KICK_STAT_HPP

#include <stat/stat_base.hpp>
#include <rummy_simulation.hpp>

namespace simu {
    namespace stat {
        using namespace simulation;

        class KickStat : public StatBase {
        public:
            virtual void operator()(const std::shared_ptr<Simulation> sim) override
            {
                auto rsim = std::static_pointer_cast<RummySimulation>(sim);

                if (!rsim->stats_enabled())
                    return;

                _create_log_file(rsim, "kicks.dat");

                if (!rsim->is_kicking()) {
                    return;
                }
                auto kicker = rsim->fish()[rsim->next_kicker_idx()];

                float dl = kicker->speed() * kicker->params().tau0 * (1. - std::exp(-kicker->tau() / kicker->params().tau0)) + kicker->params().dc;

                *_log_file << rsim->iteration() << " " << rsim->iteration() * rsim->params().timestep << " " << kicker->id() << " " << kicker->tau() << " " << kicker->speed() << " " << dl << " " << kicker->pose().yaw << std::endl;
            }
        };

    } // namespace stat
} // namespace simu

#endif
