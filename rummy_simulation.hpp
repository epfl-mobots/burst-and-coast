#ifndef RUMMY_SIMULATION_HPP
#define RUMMY_SIMULATION_HPP

#include "rummy_individual.hpp"
#include <simulation/simulation.hpp>

#include <Eigen/Core>

namespace simu {
    namespace simulation {
        struct RummyFishSettings {
            int num_timesteps = 8000;
            int num_fish = 5;
            double radius = 0.25f;
            int reference_sample_size = 10000;

            int perceived_agents = 4;
            double gamma_rand = 0.5f; // 5 fish
            double gamma_wall = 0.15f; // > 5 fish
            double wall_interaction_range = 0.06f; //
            double body_length = 0.03f;

            double alpha = 2. / 3.;
            double tau0 = 0.8f;
            double velocity_coef = 0.14f;
            double length_coef = 0.07f;
            double time_coef = 0.5f;
        };

        class RummySimulation : public Simulation {
        public:
            RummySimulation(bool stats_enable = true);

            virtual void spin_once() override;

            std::vector<RummyIndividualPtr> fish() const;
            std::vector<RummyIndividualPtr>& fish();

            RummyFishSettings& rummy_fish_settings();
            RummyFishSettings rummy_fish_settings() const;

            void reinit();

            int num_timesteps() const;
            int num_fish() const;
            double radius() const;

            int perceived_agents() const;
            double gamma_rand() const;
            double gamma_wall() const;
            double wall_interaction_range() const;
            double body_length() const;

            double alpha() const;
            double tau0() const;

            double velocity_coef() const;
            double length_coef() const;
            double time_coef() const;

            int kicking_idx() const;

        protected:
            void _init();

            int _num_timesteps;
            int _num_fish;
            double _radius;

            int _perceived_agents;
            double _gamma_rand;
            double _gamma_wall;
            double _wall_interaction_range;
            double _body_length;

            double _alpha;
            double _tau0;
            double _velocity_coef;
            double _length_coef;
            double _time_coef;

            int _kicking_idx;

            RummyFishSettings _rummy_fish_settings;
            std::vector<RummyIndividualPtr> _fish;

            Eigen::VectorXd _num_kicks;
        };

        using FishSimulationPtr = std::shared_ptr<RummySimulation>;

    } // namespace simulation
} // namespace simu

#endif
