#ifndef COSTMAP_2D_DILATION_LAYER_H_
#define COSTMAP_2D_DILATION_LAYER_H_

#include "costmap_2d/layer.h"

namespace costmap_2d
{

class DilationLayer : public costmap_2d::Layer
{

public:

    DilationLayer();

    ~DilationLayer();

    void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:

    unsigned char target_cell_value_, dilation_cell_value_;

    // The dilation radius expressed in cells
    int dilation_radius_cells_;

    // Squared distance map (unit = cells). Expresses the distance of a cell to the nearest border
    int* distance_sq_map_;
    unsigned int window_width_, window_height_;

};

} // end namespace costmap_2d

#endif
