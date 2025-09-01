#include "nav2_oaa_costmap_plugin/gradient_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_oaa_costmap_plugin
{

GradientLayer::GradientLayer()
{
  // CostmapLayer hat eigene Costmap (costmap_), die per matchSize() synchronisiert wird
}

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  declareParameter("topic", rclcpp::ParameterValue(std::string("/predictions/costmap_t1_5s")));
  node->get_parameter(name_ + ".topic", topic_);

  declareParameter("scale_to_lethal", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".scale_to_lethal", scale_to_lethal_);

  current_ = true; // Layer ist aktiv
  matchSize();     // eigene Layer-Costmap an Master anpassen

  // Abo einrichten
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(msg_mutex_);
      latest_grid_ = msg;
      // Welt-Bounds berechnen
      const auto& info = msg->info;
      const double r = info.resolution;
      const double ox = info.origin.position.x;
      const double oy = info.origin.position.y;
      grid_min_x_ = ox;
      grid_min_y_ = oy;
      grid_max_x_ = ox + r * static_cast<double>(info.width);
      grid_max_y_ = oy + r * static_cast<double>(info.height);
      new_data_ = true;
    }
  );
}

void GradientLayer::onFootprintChanged()
{
  // Keine spezielle Behandlung notwendig, aber wir markieren, dass ein Update ok ist
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
               "PredictionsLayer::onFootprintChanged(): footprint pts: %zu",
               layered_costmap_->getFootprint().size());
}

void GradientLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                 double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lk(msg_mutex_);
  if (new_data_ && latest_grid_)
  {
    // Nur die Region der letzten OG updaten
    *min_x = std::min(*min_x, grid_min_x_);
    *min_y = std::min(*min_y, grid_min_y_);
    *max_x = std::max(*max_x, grid_max_x_);
    *max_y = std::max(*max_y, grid_max_y_);
    new_data_ = false;  // Bounds gesetzt, nächster Zyklus macht Costs
  }
}

void GradientLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  nav_msgs::msg::OccupancyGrid::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(msg_mutex_);
    msg = latest_grid_;
  }
  if (!msg) return;

  // Sicherheits-Check Frame
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame)
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("nav2_costmap_2d"), *(node_.lock()->get_clock()), 3000,
      "OccupancyGrid frame_id (%s) != global_frame (%s). Werte werden ohne TF-Transform gemappt.",
      msg->header.frame_id.c_str(), global_frame.c_str());
  }

  // Fenster innerhalb der Layer-Karte klemmen
  unsigned int size_x = this->getSizeInCellsX();
  unsigned int size_y = this->getSizeInCellsY();
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  if (min_i >= max_i || min_j >= max_j) return;

  unsigned char* layer_array = this->getCharMap();

  // OG Infos
  const auto& info = msg->info;
  const double r = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const unsigned int W = info.width;
  const unsigned int H = info.height;

  // Ein-Pass: Für jedes Layer-Zellzentrum prüfe zugehörige OG-Zelle; setze NO_INFORMATION, wenn außerhalb/unknown
  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      const int layer_index = this->getIndex(i, j);

      double wx, wy;
      this->mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      // OG-Index (nearest/ceil->floor)
      const int oi = static_cast<int>(std::floor((wx - ox) / r));
      const int oj = static_cast<int>(static_cast<int>(std::floor((wy - oy) / r)));

      if (oi >= 0 && oj >= 0 && oi < static_cast<int>(W) && oj < static_cast<int>(H))
      {
        const int8_t v = msg->data[oj * W + oi];
        if (v < 0)
        {
          layer_array[layer_index] = NO_INFORMATION; // ignorieren im Merge
        }
        else
        {
          // v in [0..100] -> Costmap [0..LETHAL_OBSTACLE]
          unsigned char cost;
          if (scale_to_lethal_)
          {
            // Skaliere linear und sattle bei LETHAL_OBSTACLE
            const int scaled = static_cast<int>(std::round(v * (LETHAL_OBSTACLE / 100.0)));
            cost = static_cast<unsigned char>(std::max(0, std::min(scaled, static_cast<int>(LETHAL_OBSTACLE))));
          }
          else
          {
            // Nur 0 oder LETHAL (optional)
            cost = (v >= 100) ? LETHAL_OBSTACLE : static_cast<unsigned char>(v);
          }
          layer_array[layer_index] = cost;
        }
      }
      else
      {
        layer_array[layer_index] = NO_INFORMATION;
      }
    }
  }

  // Mit Max in den Master mergen (NO_INFORMATION wird dabei ignoriert)
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace nav2_oaa_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_oaa_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
