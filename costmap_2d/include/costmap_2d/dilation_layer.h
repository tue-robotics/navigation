#ifndef COSTMAP_2D_DILATION_LAYER_H_
#define COSTMAP_2D_DILATION_LAYER_H_

#include "costmap_2d/layer.h"
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/DilationPluginConfig.h>

namespace costmap_2d
{

class DilationLayer : public costmap_2d::Layer
{

public:

    DilationLayer();

    ~DilationLayer();

    void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void onInitialize();

private:

    unsigned char target_cell_value_, dilation_cell_value_;

    // The dilation radius expressed in meters
    double dilation_radius_;

    // Squared distance map (unit = cells). Expresses the distance of a cell to the nearest border
    int* distance_sq_map_;
    unsigned int window_width_, window_height_;

    dynamic_reconfigure::Server<costmap_2d::DilationPluginConfig> *dsrv_;
    void reconfigureCB(costmap_2d::DilationPluginConfig &config, uint32_t level);

};

} // end namespace costmap_2d

#endif
