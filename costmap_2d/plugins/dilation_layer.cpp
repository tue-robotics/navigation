#include "costmap_2d/dilation_layer.h"

#include <queue>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::DilationLayer, costmap_2d::Layer)

namespace costmap_2d
{

// ----------------------------------------------------------------------------------------------------

class CellData
{
public:

  CellData(int index_, int window_index_, int dx_, int dy_, int distance_) :
      index(index_), window_index(window_index_), dx(dx_), dy(dy_), distance_sq(distance_)
  {}

  int index;        // index in the whole costmap
  int window_index; // index in the costmap window
  short dx, dy;     // delta x and y from the source cell. (shorts are 2 bytes. Now this whole struct is only 16 bytes)
  int distance_sq;  // squared distance in cells

};

// ----------------------------------------------------------------------------------------------------

class KernelCell
{
public:

    KernelCell(int delta_index_, int delta_window_index_, int dx_, int dy_) :
        delta_index(delta_index_), delta_window_index(delta_window_index_), dx(dx_), dy(dy_)
    {}

    int delta_index;        // delta index
    int delta_window_index; // delta window index
    short dx, dy;           // delta x and y
};

// ----------------------------------------------------------------------------------------------------

DilationLayer::DilationLayer() : distance_sq_map_(0), window_width_(0), window_height_(0), dsrv_(NULL)
{
    target_cell_value_ = 255;
    dilation_cell_value_ = 255;
    dilation_radius_ = 0.01;
}

// ----------------------------------------------------------------------------------------------------

void DilationLayer::reconfigureCB(costmap_2d::DilationPluginConfig &config, uint32_t level)
{
  target_cell_value_     = config.target_cell_value;
  dilation_cell_value_   = config.dilation_cell_value;
  dilation_radius_       = config.dilation_radius;

  ROS_INFO_STREAM("Dilation:\n\tTarget cell value:   " << (int)target_cell_value_ << "\n\tDilation cell value: " << (int)dilation_cell_value_ << "\n\tDilation radius:     " << (double)dilation_radius_ << "\n");

}

void DilationLayer::onInitialize()
{
    ROS_INFO("On initialize");
    //boost::unique_lock < boost::shared_mutex > lock(*access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    //current_ = true;
    //seen_ = NULL;
    //need_reinflation_ = false;
    ROS_INFO("Defining reconfigurecallback");
    dynamic_reconfigure::Server<costmap_2d::DilationPluginConfig>::CallbackType cb = boost::bind(
        &DilationLayer::reconfigureCB, this, _1, _2);

    if(dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::DilationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
    ROS_INFO("On initialize done");
}

// ----------------------------------------------------------------------------------------------------

DilationLayer::~DilationLayer()
{
    delete distance_sq_map_;
    if(dsrv_)
        delete dsrv_;
}

// ----------------------------------------------------------------------------------------------------

void DilationLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    unsigned char* master_array = master_grid.getCharMap();

    unsigned int width = master_grid.getSizeInCellsX();
    unsigned int height = master_grid.getSizeInCellsY();

    // Calculate window width and height
    unsigned int w_width = max_i - min_i;
    unsigned int w_height = max_j - min_j;

    int dilation_radius_cells = dilation_radius_ / master_grid.getResolution();

    // Calculate the squared dilation radius (in cells)
    int dilation_radius_cells_sq = dilation_radius_cells * dilation_radius_cells;

    // Check if we need to allocate a distance map (if it not yet exists, or if the window size changed)
    if (!distance_sq_map_ || w_width != window_width_ || w_height != window_height_)
    {
        delete distance_sq_map_;
        distance_sq_map_ = new int[w_width * w_height];
        window_width_ = w_width;
        window_height_ = w_height;

        // Set the borders of the distance map to the maximum dilation distance.
        // This prevents the algorithms from accidentally dilating off the window
        for(int i = 0; i < w_width; ++i)
        {
            distance_sq_map_[i] = 0;
            distance_sq_map_[w_width * (w_height - 1) + i] = 0;
        }

        for(int j = 0; j < w_height; ++j)
        {
            distance_sq_map_[w_width * j] = 0;
            distance_sq_map_[w_width * j + (w_width - 1)] = 0;
        }
    }

    // Create kernel
    std::vector<KernelCell> kernel;
    kernel.push_back(KernelCell(-width, -w_width, 0, -1));
    kernel.push_back(KernelCell( width,  w_width, 0,  1));
    kernel.push_back(KernelCell(-1, -1, -1, 0));
    kernel.push_back(KernelCell( 1,  1,  1, 0));

    // Build Queue
    std::queue<CellData> Q;

    // Loop over all cells in the given window, and queue border cells that have the target value
    for (int j = min_j + 1; j + 1 < max_j; j++)
    {
        for (int i = min_i + 1; i + 1 < max_i; i++)
        {
            int index = master_grid.getIndex(i, j);
            unsigned char cost = master_array[index];

            // Check if cell has target cell value (the value we want to dilate)
            if (cost == target_cell_value_)
            {
                int window_index = (j - min_j) * w_width + (i - min_i);
                distance_sq_map_[window_index] = dilation_radius_cells_sq;

                // Check if the cell borders to a cell with non-target value. If so, it is a border
                // cell, so enqueue it.
                if (master_array[index - 1] != target_cell_value_ ||
                    master_array[index + 1] != target_cell_value_ ||
                    master_array[index - width] != target_cell_value_ ||
                    height && master_array[index + width] != target_cell_value_)
                {
                    Q.push(CellData(index, window_index, 0, 0, 0));
                }
            }
        }
    }

    std::vector<int> dilated_cells;

    while(!Q.empty())
    {
        const CellData& c = Q.front();
        int current_distance = distance_sq_map_[c.window_index];

        // Set the value of the current set to the new cell value
        dilated_cells.push_back(c.index);

        // Check if we have to expand
        if (c.distance_sq < current_distance)
        {
            distance_sq_map_[c.window_index] = c.distance_sq;

            // Expand
            for(unsigned int i = 0; i < kernel.size(); ++i)
            {
                const KernelCell& kc = kernel[i];
                int new_index = c.index + kc.delta_index;

                // Only dilate in the direction of target cells
                if (master_array[new_index] == target_cell_value_)
                {
                    short dx_new = c.dx + kc.dx;
                    short dy_new = c.dy + kc.dy;
                    int new_distance_sq = (dx_new * dx_new) + (dy_new * dy_new);

                    if (new_distance_sq < dilation_radius_cells_sq)
                    {
                        int new_window_index = c.window_index + kc.delta_window_index;
                        Q.push(CellData(new_index, new_window_index, dx_new, dy_new, new_distance_sq));
                    }
                }
            }
        }

        Q.pop();
    }

    for(std::vector<int>::const_iterator it = dilated_cells.begin(); it != dilated_cells.end(); ++it)
        master_array[*it] = dilation_cell_value_;
}

} // end namespace costmap_2d

