#include<costmap_2d/ed_layer.h>
#include<costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::EdLayer, costmap_2d::Layer)

//using costmap_2d::NO_INFORMATION;
//using costmap_2d::LETHAL_OBSTACLE;
//using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

EdLayer::EdLayer() : dsrv_(NULL) {}

EdLayer::~EdLayer()
{
    if(dsrv_)
        delete dsrv_;
}

void EdLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(-1));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  if (temp_lethal_threshold != -1) { std::cout << "lethal_cost_threshold is not a parameter in the ED layer!!!" << std::endl;}
  lethal_threshold_ = 2;//std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &EdLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
  
  if(subscribe_to_updates_)
  {
    ROS_INFO("Subscribing to updates");
    map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &EdLayer::incomingUpdate, this);
  }

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &EdLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void EdLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

unsigned char EdLayer::interpretValue(unsigned char value)
{

  //check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return value + 200; // Increase by 200
  else if (trinary_costmap_)
    return FREE_SPACE;
  else
      ROS_ERROR("Something went terribly wrong");

}

void EdLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))
  {
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y){
    resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      setCost(j, i, interpretValue(value));
      ++index;
    }
  }
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void EdLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int index_base = (update->y + y) * update->width;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            setCost(x, y, update->data[di++]);
        }
    }
    x_ = update->x;
    y_ = update->y;
    width_ = update->width;
    height_ = update->height;
    has_updated_data_ = true;
}

void EdLayer::activate()
{
    onInitialize();
}

void EdLayer::deactivate()
{
    map_sub_.shutdown();
    if (subscribe_to_updates_)
        map_update_sub_.shutdown();
}

void EdLayer::reset()
{
    deactivate();
    activate();
}

void EdLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
    return;
    
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;
  
  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  
  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
  
  has_updated_data_ = false;

}

void EdLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  if (!map_received_)
    return;

  if (!layered_costmap_->isRolling())
  {
    if(!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    unsigned int mx, my;
    double wx, wy;
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        if (worldToMap(wx, wy, mx, my))
        {
          unsigned char cost = getCost(mx, my);
          if (cost == NO_INFORMATION)
              continue;

          if(!use_maximum_)
            master_grid.setCost(i, j, cost);
          else
          {
            unsigned char old_cost = master_grid.getCost(i, j);
			if (track_unknown_space_)
              //if (cost == LETHAL_OBSTACLE)
              if (cost >= LETHAL_OBSTACLE)
			    master_grid.setCost(i, j, cost);
		      else
			    master_grid.setCost(i, j, std::max(cost, old_cost));
			else
			  if (old_cost == NO_INFORMATION)
			    master_grid.setCost(i, j, cost);
		      else
			    master_grid.setCost(i, j, std::max(cost, old_cost));
          }
        }
      }
    }
  }
}

}