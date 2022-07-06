#ifndef RUMMY_INDIVIDUAL_HPP
#define RUMMY_INDIVIDUAL_HPP

#include <simulation/individual.hpp>
#include <types/movement/pose2d.hpp>

#include <tools/random/random_generator.hpp>
#include <tuple>

#include <Eigen/Core>

#include <chrono>
#include <random>

namespace simu {
    namespace simulation {

        namespace defaults {
            struct RummyIndividualParams {
                float radius = 25.;

                // interactions
                int perceived_agents = 1;
                float gamma_rand = 0.3;
                float gamma_wall = 0.23;
                float gamma_sym = 0.;
                float gamma_asym = 0.05;
                float wall_interaction_range = 6.;

                float dw = 6.;
                float dc = 1.;
                float alpha_w = 0.;
                float gamma_attraction = 0.3;
                float gamma_alignment = 0.3;

                bool iuturn = true;
                float duturn = 6.;
                float pjump = 0.;
                float psi_c = 0.25;

                // kicks
                float vmean = 43.;
                float vmin = 1.;
                float vmem = 0.9;
                float vmem12 = 0.5;
                float vcut = 35.;
                float taumean = 0.52;
                float taumin = 0.2;
                float tau0 = 0.8;

                // simu
                int itermax = 1000;
            };
        } // namespace defaults

        enum Order {
            DECREASING,
            INCREASING
        };

        class RummyIndividual;
        using RummyIndividualPtr = std::shared_ptr<RummyIndividual>;
        using state_t = std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
        using const_state_t = const std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>&;

        class RummyIndividual : public Individual<double, double> {
        public:
            RummyIndividual(int id, defaults::RummyIndividualParams params = defaults::RummyIndividualParams());
            virtual ~RummyIndividual();

            virtual void glide(const std::shared_ptr<Simulation> sim);
            virtual void prepare_kick(const std::shared_ptr<Simulation> sim);
            virtual void kick(const std::shared_ptr<Simulation> sim);

            virtual void stimulate(const std::shared_ptr<Simulation> sim) override;
            virtual void move(const std::shared_ptr<Simulation> sim) override;

            float kick_duration() const;
            float t0() const;
            float& t0();
            float tau() const;
            float& tau();
            Pose2d<float> kick_pose() const;
            float speed() const;
            Pose2d<float> pose() const;
            Pose2d<float>& pose();
            Pose2d<float> traj_pose() const;

        protected:
            std::tuple<float, float> compute_interactions(const state_t& state, std::vector<int> neighs, const std::shared_ptr<Simulation> sim);
            state_t compute_state(const std::vector<RummyIndividualPtr>& fish) const;
            std::vector<int> sort_neighbours(
                const Eigen::VectorXd& values, const int kicker_idx, Order order = Order::INCREASING) const;

            double angle_to_pipi(double difference) const;

            defaults::RummyIndividualParams _params;
            Position<position_type_t> _desired_position;
            Speed<speed_type_t> _desired_speed;

            uint64_t _num_jumps;
            uint64_t _num_kicks;
            uint64_t _num_uturn;

            float _t0;
            float _tau;
            Pose2d<float> _pose;
            Pose2d<float> _traj_pose;
            Pose2d<float> _kick_pose;
            float _speed;
            float _traj_speed;
        };

    } // namespace simulation
} // namespace simu

#endif
