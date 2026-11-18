#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hri_msgs/msg/facial_landmarks.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class KinesicsAnalyzerNode : public rclcpp::Node {
public:
  KinesicsAnalyzerNode() : Node("kinesics_analyzer_node") {
    this->declare_parameter<double>("crossed_arms_ratio", 0.65);
    this->declare_parameter<double>("face_touch_ratio", 0.55);

    auto qos = rclcpp::QoS(10);

    // Suscribirse a las listas de cuerpos, caras y personas rastreadas
    bodies_tracked_sub_ = this->create_subscription<hri_msgs::msg::IdsList>(
        "/humans/bodies/tracked", qos,
        std::bind(&KinesicsAnalyzerNode::on_bodies_tracked, this,
                  std::placeholders::_1));

    faces_tracked_sub_ = this->create_subscription<hri_msgs::msg::IdsList>(
        "/humans/faces/tracked", qos,
        std::bind(&KinesicsAnalyzerNode::on_faces_tracked, this,
                  std::placeholders::_1));

    persons_tracked_sub_ = this->create_subscription<hri_msgs::msg::IdsList>(
        "/humans/persons/tracked", qos,
        std::bind(&KinesicsAnalyzerNode::on_persons_tracked, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&KinesicsAnalyzerNode::process_kinesics, this));

    RCLCPP_INFO(this->get_logger(),
                "KinesicsAnalyzerNode iniciado (modo directo seguro sin segfault).");
  }

private:
  double dist2d(float x1, float y1, float x2, float y2) const {
    return std::hypot(static_cast<double>(x1 - x2),
                      static_cast<double>(y1 - y2));
  }

  void on_bodies_tracked(const hri_msgs::msg::IdsList::SharedPtr msg) {
    tracked_bodies_ = msg->ids;
    for (const auto &body_id : msg->ids) {
      if (skeleton_subs_.find(body_id) == skeleton_subs_.end()) {
        std::string topic = "/humans/bodies/" + body_id + "/skeleton2d";
        skeleton_subs_[body_id] =
            this->create_subscription<hri_msgs::msg::Skeleton2D>(
                topic, 10,
                [this, body_id](const hri_msgs::msg::Skeleton2D::SharedPtr skel) {
                  latest_skeletons_[body_id] = *skel;
                });
      }
    }
  }

  void on_faces_tracked(const hri_msgs::msg::IdsList::SharedPtr msg) {
    tracked_faces_ = msg->ids;
    for (const auto &face_id : msg->ids) {
      if (landmarks_subs_.find(face_id) == landmarks_subs_.end()) {
        std::string topic = "/humans/faces/" + face_id + "/landmarks";
        landmarks_subs_[face_id] =
            this->create_subscription<hri_msgs::msg::FacialLandmarks>(
                topic, 10,
                [this, face_id](
                    const hri_msgs::msg::FacialLandmarks::SharedPtr lms) {
                  latest_landmarks_[face_id] = *lms;
                });
      }
    }
  }

  void on_persons_tracked(const hri_msgs::msg::IdsList::SharedPtr msg) {
    tracked_persons_ = msg->ids;
    for (const auto &person_id : msg->ids) {
      if (person_body_subs_.find(person_id) == person_body_subs_.end()) {
        std::string b_topic = "/humans/persons/" + person_id + "/body_id";
        person_body_subs_[person_id] =
            this->create_subscription<std_msgs::msg::String>(
                b_topic, 10,
                [this, person_id](const std_msgs::msg::String::SharedPtr s) {
                  person_to_body_[person_id] = s->data;
                });

        std::string f_topic = "/humans/persons/" + person_id + "/face_id";
        person_face_subs_[person_id] =
            this->create_subscription<std_msgs::msg::String>(
                f_topic, 10,
                [this, person_id](const std_msgs::msg::String::SharedPtr s) {
                  person_to_face_[person_id] = s->data;
                });
      }
    }
  }

  void analyze_pair(const std::string &target_id, const std::string &body_id,
                    const std::string &face_id) {
    double crossed_ratio =
        this->get_parameter("crossed_arms_ratio").as_double();
    double touch_ratio = this->get_parameter("face_touch_ratio").as_double();

    bool crossed_arms = false;
    bool hand_raised = false;
    bool face_touch = false;

    auto skel_it = latest_skeletons_.find(body_id);
    if (skel_it != latest_skeletons_.end()) {
      const auto &skel = skel_it->second.skeleton;
      using Skel = hri_msgs::msg::Skeleton2D;

      if (skel.size() > Skel::RIGHT_WRIST) {
        const auto &l_shoulder = skel[Skel::LEFT_SHOULDER];
        const auto &r_shoulder = skel[Skel::RIGHT_SHOULDER];
        const auto &l_elbow = skel[Skel::LEFT_ELBOW];
        const auto &r_elbow = skel[Skel::RIGHT_ELBOW];
        const auto &l_wrist = skel[Skel::LEFT_WRIST];
        const auto &r_wrist = skel[Skel::RIGHT_WRIST];

        double shoulder_width =
            dist2d(l_shoulder.x, l_shoulder.y, r_shoulder.x, r_shoulder.y);

        if (shoulder_width > 1e-4) {
          // 1. Brazos cruzados
          double d_lw_re = dist2d(l_wrist.x, l_wrist.y, r_elbow.x, r_elbow.y);
          double d_rw_le = dist2d(r_wrist.x, r_wrist.y, l_elbow.x, l_elbow.y);

          if ((d_lw_re / shoulder_width) < crossed_ratio &&
              (d_rw_le / shoulder_width) < crossed_ratio) {
            crossed_arms = true;
          }

          // 2. Brazo levantado
          if ((l_wrist.y < l_shoulder.y - 0.15f * shoulder_width) ||
              (r_wrist.y < r_shoulder.y - 0.15f * shoulder_width)) {
            hand_raised = true;
          }

          // 3. Auto-contacto mano-cara / ceja
          auto lms_it = latest_landmarks_.find(face_id);
          if (lms_it != latest_landmarks_.end()) {
            for (const auto &lm : lms_it->second.landmarks) {
              if (lm.c < 0.3f) {
                continue;
              }
              double d_left = dist2d(l_wrist.x, l_wrist.y, lm.x, lm.y);
              double d_right = dist2d(r_wrist.x, r_wrist.y, lm.x, lm.y);

              if ((d_left / shoulder_width) < touch_ratio ||
                  (d_right / shoulder_width) < touch_ratio) {
                face_touch = true;
                break;
              }
            }
          }
        }
      }
    }

    std::string topic_name = "/humans/persons/" + target_id + "/kinesics";
    if (publishers_.find(target_id) == publishers_.end()) {
      publishers_[target_id] =
          this->create_publisher<std_msgs::msg::String>(topic_name, 10);
      RCLCPP_INFO(this->get_logger(), "Publicando kinesics en: %s",
                  topic_name.c_str());
    }

    std_msgs::msg::String out;
    out.data = "{\"person_id\": \"" + target_id +
               "\", \"crossed_arms\": " + (crossed_arms ? "true" : "false") +
               ", \"hand_raised\": " + (hand_raised ? "true" : "false") +
               ", \"face_touch\": " + (face_touch ? "true" : "false") + "}";
    publishers_[target_id]->publish(out);
  }

  void process_kinesics() {
    // Si hay personas fusionadas en person_manager, usamos su ID
    if (!tracked_persons_.empty()) {
      for (const auto &pid : tracked_persons_) {
        std::string bid = person_to_body_.count(pid) ? person_to_body_[pid] : "";
        std::string fid = person_to_face_.count(pid) ? person_to_face_[pid] : "";
        analyze_pair(pid, bid, fid);
      }
    } else if (!tracked_bodies_.empty()) {
      // Fallback directo por body_id si person_manager aún no ha fusionado
      std::string fid = tracked_faces_.empty() ? "" : tracked_faces_.front();
      for (const auto &bid : tracked_bodies_) {
        analyze_pair(bid, bid, fid);
      }
    }
  }

  rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr bodies_tracked_sub_;
  rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr faces_tracked_sub_;
  rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr persons_tracked_sub_;

  std::vector<std::string> tracked_bodies_;
  std::vector<std::string> tracked_faces_;
  std::vector<std::string> tracked_persons_;

  std::unordered_map<std::string,
                     rclcpp::Subscription<hri_msgs::msg::Skeleton2D>::SharedPtr>
      skeleton_subs_;
  std::unordered_map<
      std::string,
      rclcpp::Subscription<hri_msgs::msg::FacialLandmarks>::SharedPtr>
      landmarks_subs_;
  std::unordered_map<std::string,
                     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>
      person_body_subs_;
  std::unordered_map<std::string,
                     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>
      person_face_subs_;

  std::unordered_map<std::string, hri_msgs::msg::Skeleton2D> latest_skeletons_;
  std::unordered_map<std::string, hri_msgs::msg::FacialLandmarks>
      latest_landmarks_;
  std::unordered_map<std::string, std::string> person_to_body_;
  std::unordered_map<std::string, std::string> person_to_face_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string,
                     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>
      publishers_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinesicsAnalyzerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}