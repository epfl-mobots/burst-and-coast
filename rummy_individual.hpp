#ifndef RUMMY_INDIVIDUAL_HPP
#define RUMMY_INDIVIDUAL_HPP

#include <simulation/individual.hpp>

#include <tools/random/random_generator.hpp>
#include <tuple>

#include <Eigen/Core>

namespace simu {
    namespace simulation {
        struct FishParams {

            void print() const
            {
            }
        };

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
            RummyIndividual();
            RummyIndividual(const FishParams& params);
            virtual ~RummyIndividual();

            virtual void stimulate(const std::shared_ptr<Simulation> sim) override;
            virtual void interact(const std::shared_ptr<Simulation> sim);
            virtual void move(const std::shared_ptr<Simulation> sim) override;

            void stepper(const std::shared_ptr<Simulation> sim);
            std::tuple<double, double> model_stepper(double radius) const;
            void free_will(const std::shared_ptr<Simulation> sim, const_state_t state, const std::tuple<double, double>& model_out, const std::vector<int>& idcs);

            FishParams& fish_params();
            FishParams fish_params() const;

            Speed<speed_type_t> desired_speed() const { return _desired_speed; }
            Speed<speed_type_t>& desired_speed() { return _desired_speed; }

            Position<position_type_t> desired_position() const { return _desired_position; }
            Position<position_type_t>& desired_position() { return _desired_position; }

            double time_kicker() const;

            double time() const;
            double& time();
            double angular_direction() const;
            double& angular_direction();
            double tau() const;
            double& tau();
            double kick_length() const;
            double& kick_length();
            double peak_velocity() const;
            double& peak_velocity();

            bool& is_kicking();
            bool is_kicking() const;

        protected:
            double wall_distance_interaction(double gamma_wall, double wall_interaction_range, double ag_radius, double radius) const;
            double wall_angle_interaction(double theta) const;

            double wall_distance_attractor(double distance, double radius) const;
            double wall_perception_attractor(double perception) const;
            double wall_angle_attractor(double phi) const;

            double alignment_distance_attractor(double distance, double radius) const;
            double alignment_perception_attractor(double perception) const;
            double alignment_angle_attractor(double phi) const;

            state_t compute_state(const std::vector<RummyIndividualPtr>& fish) const;
            std::vector<int> sort_neighbours(Eigen::VectorXd values, const int kicker_idx, Order order = Order::INCREASING) const;

            double angle_to_pipi(double difference) const;

            FishParams _fish_params;
            Position<position_type_t> _desired_position;
            Speed<speed_type_t> _desired_speed;

            bool _is_kicking;

            double _angular_direction;
            double _peak_velocity;
            double _kick_length;
            double _time;
            double _tau;
        };

    } // namespace simulation
} // namespace simu

#endif
