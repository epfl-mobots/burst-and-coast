#ifndef KICK_COUNTER_STAT_HPP
#define KICK_COUNTER_STAT_HPP

#include <stat/stat_base.hpp>
#include <rummy_simulation.hpp>

#include <complex>

namespace simu {
    namespace stat {
        using namespace simulation;

        class KickCounter : public StatBase {
        public:
            virtual void operator()(const std::shared_ptr<Simulation> sim) override
            {
                auto rsim = std::static_pointer_cast<RummySimulation>(sim);

                if (!rsim->stats_enabled())
                    return;

                _create_log_file(rsim, "kick_counter.dat");

                for (size_t i = 0; i < rsim->fish().size(); ++i) {
                    *_log_file << rsim->iteration() << " ";
                    for (const RummyIndividualPtr& f : rsim->fish())
                        *_log_file << f->is_kicking() << " ";
                    *_log_file << std::endl;
                }
            }
        };

    } // namespace stat
} // namespace simu

#endif
