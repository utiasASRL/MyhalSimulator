#include "flowfield.hh"




FlowField::FlowField(boost::shared_ptr<Costmap> costmap0, ignition::math::Vector3d goal0, double obstacle_range0, double obstacle_strength0)
{
    // Init variables
    obstacle_range = obstacle_range0;
    obstacle_strength = obstacle_strength0;
    resolution = costmap0->resolution;
    cols = costmap0->cols;
    rows = costmap0->rows;
    goal.X() = goal0.X();
    goal.Y() = goal0.Y();
    boundary = costmap0->boundary;
    field = std::vector<std::vector<ignition::math::Vector2d>>(rows, std::vector<ignition::math::Vector2d>(cols));
    value_function = std::vector<std::vector<double>>(rows, std::vector<double>(cols));
    obstacle_map = std::vector<std::vector<double>>(rows, std::vector<double>(cols));
}


FlowField::FlowField()
{
    // Init variables
    obstacle_range = 0;
    obstacle_strength = 0;
    resolution = 0;
    cols = 0;
    rows = 0;
    goal.X() = 0;
    goal.Y() = 0;
    field = std::vector<std::vector<ignition::math::Vector2d>>(1, std::vector<ignition::math::Vector2d>(1));
    value_function = std::vector<std::vector<double>>(1, std::vector<double>(1));
}


bool FlowField::PosToIndicies(ignition::math::Vector3d pos, int &r, int &c)
{
    r = (int)floor((boundary.Max().Y() - pos.Y()) / resolution);
    c = (int)floor((pos.X() - boundary.Min().X()) / resolution);
    return utilities::inside_box(boundary, pos, true);
}

bool FlowField::IndiciesToPos(ignition::math::Vector3d &pos, int r, int c)
{

    pos = ignition::math::Vector3d(boundary.Min().X() + c * resolution, boundary.Max().Y() - r * resolution, 0);
    return ((r >= 0 && r < rows) && (c >= 0 && c < cols));
}


std::vector<std::vector<int>> FlowField::GetNeighbours(std::vector<int> curr_ind, bool diag)
{
    std::vector<std::vector<int>> res;

    if (curr_ind[0] > 0){ // we can return top
        res.push_back({curr_ind[0]-1, curr_ind[1]});
    }

    if (curr_ind[1] > 0){ // we can return left
        res.push_back({curr_ind[0], curr_ind[1]-1});
    }

    if (curr_ind[0] < rows-1){ // we can return bot
        res.push_back({curr_ind[0]+1, curr_ind[1]});
    }

    if (curr_ind[1] < cols-1){ // we can return right
        res.push_back({curr_ind[0], curr_ind[1]+1});
    }

    if (diag){
        if (curr_ind[0] > 0 && curr_ind[1] > 0){ // we can return top left
            res.push_back({curr_ind[0]-1,curr_ind[1]-1});
        }

        if (curr_ind[0] > 0 && curr_ind[1] <cols-1){ // we can return bottom left
            res.push_back({curr_ind[0]-1,curr_ind[1]+1});
        }

        if (curr_ind[0] < rows-1 && curr_ind[1] > 0){ // we can return top right
            res.push_back({curr_ind[0]+1,curr_ind[1]-1});
        }

        if (curr_ind[0] < rows-1 && curr_ind[1] < cols-1){ // we can return bottom right
            res.push_back({curr_ind[0]+1,curr_ind[1]+1});
        }
    }

    return res;
}



/*
* Create an internal costmap with smooth increase in the direction of wals and obstacles. To avoid the "hug wall" effect
*/
void FlowField::ObstacleMap(std::vector<std::vector<int>>& costmap)
{
    
    // Init: the cost of a cell is equal to the size of the cell in the real world
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            obstacle_map[r][c] = resolution;
        }
    }

    // Get the range of obstacle repulsive flow in pixels
    int pixel_range = (int)floor(obstacle_range / resolution ) + 1;

    // tmp variable
    double exp_factor = resolution * resolution / (0.5 * 0.5 * obstacle_range * obstacle_range);

    // Loop over all costmap position
    for (int c = 0; c < cols; ++c)
    {
        // Kernel column range (according to costmap boundaries)
        int c1 = -pixel_range;
        if (c + c1 < 0)
            c1 = -c;
        int c2 = pixel_range;
        if (c + c2 > cols-1)
            c2 = cols - 1 - c;

        for (int r = 0; r < rows; r++)
        {
            if (costmap[r][c] >=255)
            {
                // Kernel row range (according to costmap boundaries)
                int r1 = -pixel_range;
                if (r + r1 < 0)
                    r1 = -r;
                int r2 = pixel_range;
                if (r + r2 > rows-1)
                    r2 = rows - 1 - r;

                // Inside obstacles, the cost is infinite
                obstacle_map[r][c] = 10e9;

                // The closer to an obstacle the higher the cost
                for (int cc = c1; cc <= c2; cc++)
                {
                    for (int rr = r1; rr <= r2; rr++)
                    {
                        double dist2 = rr * rr + cc * cc;
                        double new_value = resolution * (1.0 + obstacle_strength * exp(-dist2 * exp_factor));
                        if (new_value > obstacle_map[r + rr][c + cc])
                            obstacle_map[r + rr][c + cc] = new_value;
                    }
                }
            }
        }
    }
}


/*
* Integrate initialized the integration field at 10e9. It sets the goal position to 0 in the field, then iteratively calculate the value of the integration field from the goal following the algorithm here
* https://leifnode.com/2013/12/flow-field-pathfinding/
*
*/
bool FlowField::Integrate(std::vector<std::vector<int>>& costmap)
{
    // First get the obstacle map
    ObstacleMap(costmap);

    // Init variables
    double sqrt2 = sqrt(2);

    // Init integration field to very high value
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            value_function[r][c] = 10e9;
        }   
    }
    
    // Check if goal is in boundary
    if (goal.X() < boundary.Min().X() || goal.X() > boundary.Max().X() || goal.Y() >  boundary.Max().Y() || goal.Y() < boundary.Min().Y()){
        return false;
    }

    // Get position of the goal in the costmap
    int goal_r, goal_c;
    PosToIndicies(ignition::math::Vector3d(goal.X(), goal.Y(), 0), goal_r, goal_c);

    // Set value for goal at 0
    value_function[goal_r][goal_c] = 0;

    // Init the region growing container
    std::vector<std::vector<int>> open_list;
    open_list.push_back({goal_r, goal_c});

    // Grow region
    while (open_list.size() > 0)
    {
        // Get current candidate. This could be better implemented with pop()
        auto curr_ind = open_list.front();
        open_list.erase(open_list.begin());


        // Get list of neighbors of the candidate
        auto neighbours = GetNeighbours(curr_ind, true);

        // Deal with all neighbors
        for (auto n: neighbours)
        {
            // Init cost at the current value
            double n_cost = value_function[curr_ind[0]][curr_ind[1]];
            
            // Add cost of getting to a costmap pixel.
            if ((n[0] - curr_ind[0]) * (n[1] -curr_ind[1]) == 0)
            {
                n_cost += obstacle_map[n[0]][n[1]];
            }
            else
            {
                n_cost += sqrt2 * obstacle_map[n[0]][n[1]];
            }

            // If we are not in an obstacle and the computed cost is lower than current cost, update it
            if (n_cost < value_function[n[0]][n[1]] && obstacle_map[n[0]][n[1]] < 10e8)
            {
                value_function[n[0]][n[1]] = n_cost;
                if (std::find(open_list.begin(), open_list.end(), n) == open_list.end())
                {
                    open_list.push_back(n); 
                }
            }
        }
    }
    return true;
}


/*
* ComputeFlowFieldFine compute a flow field offset (difference in integration field between current grid pose and lowest neighbouring integration field) and angle at each point of the costmap to a given goal.
*  This finer version computes the local gradient value in the integration map
* 
* @param end goal to reach
*/
void FlowField::Compute(std::vector<std::vector<int>>& costmap)
{
    // Init variables
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;

    if (!Integrate(costmap)){
        std::cout << "ERROR INTEGRATING COSTMAP INTEGRATION FIELD." << std::endl;
        return;
    }

    int goal_r, goal_c;
    PosToIndicies(ignition::math::Vector3d(goal.X(), goal.Y(), 0), goal_r, goal_c);

    // We get gradient with Farid and Simocelly filter
    int farid_n = 2;
    double farid_5_k[5]= {0.030320,  0.249724,  0.439911,  0.249724,  0.030320};
    double farid_5_d[5]= {0.104550,  0.292315,  0.000000, -0.292315, -0.104550};
    double farid_5_dd[5]= {-0.104550,  -0.292315,  0.000000, 0.292315, 0.104550};

    // Init convolve derivative
    field = std::vector<std::vector<ignition::math::Vector2d>>(rows, std::vector<ignition::math::Vector2d>(cols));
    std::vector<std::vector<double>> value_func_2(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dx_tmp(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dy_tmp(rows, std::vector<double>(cols, 0));

    // First get rid of the 10e9 values in range of the convolution kernel. Set them to the highest value in the kernel range
    for (int c = 0; c<cols; c++)
    {
        int c1 = -farid_n;
        if (c + c1 < 0)
            c1 = -c;

        int c2 = farid_n;
        if (c + c2 > cols-1)
            c2 = cols - 1 - c;

        for (int r = 0; r<rows; r++)
        {
            int r1 = -farid_n;
            if (r + r1 < 0)
                r1 = -r;

            int r2 = farid_n;
            if (r + r2 > rows-1)
                r2 = rows - 1 - r;

            double v0 = value_function[r][c];
            if (v0 < 10e8)
            {
                value_func_2[r][c] = v0;
            }
            else
            {
                double max_v = 0;
                for (int cc = c1; cc <= c2; cc++)
                {
                    for (int rr = r1; rr <= r2; rr++)
                    {
                        double v = value_function[r + rr][c + cc];
                        if (v < 10e8 && v > max_v)
                            max_v = v;
                    }
                }
                value_func_2[r][c] = max_v;
            }
        }
    }

    // Convolution along columns
    for (int c = 0; c<cols; c++)
    {
        int b1 = -farid_n;
        if (c + b1 < 0)
            b1 = -c;

        int b2 = farid_n;
        if (c + b2 > cols-1)
            b2 = cols - 1 - c;

        for (int r = 0; r<rows; r++)
        {

            double dx = 0.0;
            double dy = 0.0;
            double v0 = value_func_2[r][c];

            // First convolve each column
            for (int b = b1; b <= b2; b++)
            {
                double integration_v = value_func_2[r][c + b];
                dx += integration_v * farid_5_d[b - b1];
                dy += integration_v * farid_5_k[b - b1];
            }
            dx_tmp[r][c] = dx;
            dy_tmp[r][c] = dy;
        }
    }

    // Second convolution along rows
    for (int r = 0; r < rows; r++)
    {
        int b1 = -farid_n;
        if (r + b1 < 0)
            b1 = -r;

        int b2 = farid_n;
        if (r + b2 > rows-1)
            b2 = rows - 1 - r;

        for (int c = 0; c < cols; c++)
        {
            double dx = 0.0;
            double dy = 0.0;

            // First convolve each column
            for (int b = b1; b <= b2; b++)
            {
                dx += dx_tmp[r + b][c] * farid_5_k[b - b1];
                dy += dy_tmp[r + b][c] * farid_5_dd[b - b1];
            }
            
            // Only update outside obstacles
            if (value_function[r][c] < 10e8)
                field[r][c] = ignition::math::Vector2d(dx, dy);
        }
    }
}


bool FlowField::Lookup(ignition::math::Vector3d pos, ignition::math::Vector2d &res)
{
    int row_num, col_num;
    if(!PosToIndicies(pos, row_num, col_num)){
        return false;
    }
    res = field[row_num][col_num];
    return true;
}

bool FlowField::SmoothLookup(ignition::math::Vector3d pos, ignition::math::Vector2d &res)
{
    int r2, c2;
    if(!PosToIndicies(pos + ignition::math::Vector3d(0.5 * resolution, -0.5 * resolution, 0), r2, c2))
        return false;

    double avg_tot = 0;
    ignition::math::Vector2d avg_flow(0, 0);

    for (int r = r2 - 1; r < r2 + 1; r++)
    {
        for (int c = c2 - 1; c < c2 + 1; c++)
        {
            
            if (value_function[r][c] < 10e8)
            {
                double pix_x = (double)c * resolution + boundary.Min().X() + resolution / 2;
                double pix_y = boundary.Max().Y() - (double)r * resolution - resolution / 2;
                double weight = resolution - (pos - ignition::math::Vector3d(pix_x, pix_y, pos.Z())).Length();
                if (weight > 0)
                {
                    avg_flow += weight * field[r][c];
                    avg_tot += weight;
                }
            }
        }
    }
    
    if (avg_tot > 0)
        res = avg_flow / avg_tot;
    else
        res = avg_flow;
        return true;

}



double FlowField::Reachability()
{
    // Sum all reachable values
    int reachable_sum = 0;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (value_function[r][c] < 10e8)
                reachable_sum++;
        }
    }
    return (double)reachable_sum / (double)(rows * cols);
}





