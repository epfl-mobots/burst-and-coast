#include "rummy_simulation.hpp"

#include <tools/random/random_generator.hpp>
#include <numeric>

namespace simu {
    namespace simulation {
        RummySimulation::RummySimulation(defaults::RummySimuParams params)
            : Simulation(params), _params(params)
        {
            _init();
        }

        void RummySimulation::_init()
        {
            _iteration = 0;
            reinit();
        }

        void RummySimulation::reinit()
        {
            _fish.clear();
            _fish.resize(static_cast<size_t>(_params.num_fish));
            for (size_t i = 0; i < _fish.size(); ++i) {
                _fish[i] = std::make_shared<RummyIndividual>(i);
            }

            // log the initial position
            _update_stats(std::make_shared<RummySimulation>(*this));
        }

        void RummySimulation::spin_once()
        {
            _t_next_kick = std::numeric_limits<float>::max();
            for (size_t i = 0; i < _fish.size(); ++i) {
                if ((_fish[i]->t0() + _fish[i]->kick_duration()) < _t_next_kick) {
                    _t_next_kick = _fish[i]->t0() + _fish[i]->kick_duration();
                    _next_kicker_idx = i;
                }
            }

            // no fish kicking yet
            if ((_iteration + 1) * _sim_settings.timestep <= _t_next_kick) {
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i]->glide(std::make_shared<RummySimulation>(*this));
                }

                // update statistics
                _update_stats(std::make_shared<RummySimulation>(*this));
                _update_descriptors(std::make_shared<RummySimulation>(*this));

                Simulation::spin_once();
            }
            // compute new kick
            else {
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i]->prepare_kick(std::make_shared<RummySimulation>(*this));
                }
                _fish[_next_kicker_idx]->kick(std::make_shared<RummySimulation>(*this));
            }
        }

        std::vector<RummyIndividualPtr> RummySimulation::fish() const { return _fish; }
        std::vector<RummyIndividualPtr>& RummySimulation::fish() { return _fish; }

        float RummySimulation::t_next_kick() const
        {
            return _t_next_kick;
        }

        defaults::RummySimuParams RummySimulation::params() const
        {
            return _params;
        }

        int RummySimulation::next_kicker_idx() const
        {
            return _next_kicker_idx;
        }

    } // namespace simulation
} // namespace simu
