#ifndef RUMMY_SIMULATION_HPP
#define RUMMY_SIMULATION_HPP

#include "rummy_individual.hpp"
#include <simulation/simulation.hpp>

#include <Eigen/Core>

namespace simu {
    namespace simulation {
        namespace defaults {
            struct RummySimuParams : public Settings {
                int num_fish = 2;
                float radius = 25.;
            };
        } // namespace defaults

        class RummySimulation : public Simulation {
        public:
            RummySimulation(defaults::RummySimuParams params = defaults::RummySimuParams());

            virtual void spin_once() override;

            std::vector<RummyIndividualPtr> fish() const;
            std::vector<RummyIndividualPtr>& fish();

            int next_kicker_idx() const;
            float t_next_kick() const;
            bool is_kicking() const;
            const defaults::RummySimuParams params() const;
            defaults::RummySimuParams& params();

            void reinit();

        protected:
            defaults::RummySimuParams _params;

            std::vector<RummyIndividualPtr> _fish;

            int _next_kicker_idx;
            float _t_next_kick;
            bool _is_kicking;
        };

        using FishSimulationPtr = std::shared_ptr<RummySimulation>;

    } // namespace simulation
} // namespace simu

#endif
